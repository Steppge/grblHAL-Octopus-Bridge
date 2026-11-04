/*
  Octopus-Bridge - ESP32-S3-DevKitC-1 N16R8 at the BTT Octopus Pro

  Connects the grblHAL display (ESP32-P4) to the controller, over a USB cable or WiFi:
    display link (stream 1)   UART1 GPIO11 TX -> J26-7 (PD6 RX2), GPIO12 RX <- J26-9 (PD5 TX2)
    gamepad (stream 0)        UART2 GPIO13 TX -> TFT-4 (PA10 RX1), GPIO14 RX <- TFT-3 (PA9 TX1)
  both 921600 baud.

  USB: the S3 is USB host on its "USB" socket (GPIO19 D- / GPIO20 D+), the display a USB device with two
  CDC-ACM channels, interface 0 = display link, interface 2 = gamepad.

  WiFi: the S3 is the hidden access point of a machine network of its own (AP_SSID, a random password made on
  the first start). Display link over TCP (LINK_PORT, one client), gamepad lines over UDP (GAMEPAD_PORT,
  each line one datagram, the newest state wins). Once the display is plugged in, the S3 sends it the
  name and password of the network over the link channel ($BW line), so it can connect without cable.
  While the display is plugged in, USB is used and the access point is off.

  Without the display the UARTs stay silent, the watchdogs in grblHAL stop jogging and hold a job.

  Console on the "UART" USB-C (CH343, 115200): status, network name and password, and a pass-through of
  one UART to the console for tests without the display (Ctrl+] ends it).

  RGB LED: red no display, blue over USB, cyan over WiFi, a short green flash a second while data flows,
  violet pass-through.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "led_strip.h"

#define BRIDGE_VERSION  "0.3"

#define AP_SSID         "grblHAL-Machine"
#define AP_CHANNEL      6
#define AP_PASS_LEN     16
#define LINK_PORT       5000        // TCP: display link
#define GAMEPAD_PORT    5001        // UDP: gamepad lines
#define NET_TIMEOUT_MS  3000        // nothing from the display over WiFi for this long = gone (heartbeat 500 ms)
#define NET_TX_MS       100         // max. wait for a TCP send to the display

#define BRIDGE_BAUD     921600
#define UART_BUF        4096
#define USB_BUF         2048
#define USB_TX_MS       100         // max. wait for a USB transfer to the display

#define CONSOLE_UART    UART_NUM_0
#define PASS_EXIT       0x1D        // Ctrl+] ends the pass-through

#define LED_GPIO        48          // WS2812: GPIO48 on DevKitC-1 v1.0, GPIO38 on v1.1
#define LED_DATA_MS     50          // green flash once a second while data flows

static const char *TAG = "bridge";

typedef struct {
    const char *name;
    uart_port_t uart;
    int tx, rx;
    uint8_t iface;                  // CDC interface number on the display
    cdc_acm_dev_hdl_t dev;          // NULL while not connected
    SemaphoreHandle_t lock;         // guards dev
    uint32_t to_ctrl, to_disp;      // bytes forwarded
} channel_t;

static channel_t ch[2] = {
    { .name = "link",    .uart = UART_NUM_1, .tx = 11, .rx = 12, .iface = 0 },
    { .name = "gamepad", .uart = UART_NUM_2, .tx = 13, .rx = 14, .iface = 2 },
};

static volatile int pass = -1;      // channel passed through to the console, -1 = none
static volatile TickType_t activity;
static SemaphoreHandle_t gone;      // a display channel was unplugged
static led_strip_handle_t led;

static char ap_pass[AP_PASS_LEN + 1];
static volatile int tcp_client = -1;            // display link over WiFi, -1 = none
static SemaphoreHandle_t tcp_lock;              // guards tcp_client
static volatile TickType_t tcp_seen, udp_seen;  // last data from the display
static int udp_sock = -1;
static struct sockaddr_in udp_peer;             // the display, from its last gamepad datagram

static bool usb_plugged (void)
{
    return ch[0].dev || ch[1].dev;
}

static bool recent (TickType_t t)
{
    return t && xTaskGetTickCount() - t < pdMS_TO_TICKS(NET_TIMEOUT_MS);
}

static void forward_to_ctrl (channel_t *c, const uint8_t *data, size_t len)
{
    if(pass != c - ch) {
        uart_write_bytes(c->uart, data, len);
        c->to_ctrl += len;
        activity = xTaskGetTickCount();
    }
}

static void tcp_close (void)
{
    xSemaphoreTake(tcp_lock, portMAX_DELAY);
    if(tcp_client >= 0) {
        close(tcp_client);
        tcp_client = -1;
        ESP_LOGI(TAG, "link: display left WiFi");
    }
    xSemaphoreGive(tcp_lock);
}

// controller -> display over WiFi: the link as a byte stream, the gamepad as one datagram per line
static void net_send (channel_t *c, const uint8_t *data, size_t len)
{
    if(c == &ch[0]) {
        bool ok = true;
        xSemaphoreTake(tcp_lock, portMAX_DELAY);
        if(tcp_client >= 0) {
            ok = send(tcp_client, data, len, 0) == (int)len;
            if(ok) {
                c->to_disp += len;
                activity = xTaskGetTickCount();
            }
        }
        xSemaphoreGive(tcp_lock);
        if(!ok)
            tcp_close();
    } else {
        static char line[128];
        static size_t n = 0;
        for(size_t i = 0; i < len; i++) {
            if(n < sizeof(line))
                line[n++] = data[i];
            if(data[i] == '\n') {
                if(udp_sock >= 0 && recent(udp_seen) && n < sizeof(line)) {
                    sendto(udp_sock, line, n, 0, (struct sockaddr *)&udp_peer, sizeof(udp_peer));
                    c->to_disp += n;
                    activity = xTaskGetTickCount();
                }
                n = 0;
            }
        }
    }
}

// display -> controller
static bool usb_rx (const uint8_t *data, size_t len, void *arg)
{
    forward_to_ctrl((channel_t *)arg, data, len);

    return true;
}

static void usb_event (const cdc_acm_host_dev_event_data_t *event, void *arg)
{
    channel_t *c = arg;

    switch(event->type) {

        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            xSemaphoreTake(c->lock, portMAX_DELAY);
            cdc_acm_host_close(event->data.cdc_hdl);
            c->dev = NULL;
            xSemaphoreGive(c->lock);
            ESP_LOGI(TAG, "%s: display unplugged", c->name);
            xSemaphoreGive(gone);
            break;

        case CDC_ACM_HOST_ERROR:
            ESP_LOGW(TAG, "%s: USB error %d", c->name, event->data.error);
            break;

        default:
            break;
    }
}

// controller -> display, or the console during a pass-through
static void uart_task (void *arg)
{
    channel_t *c = arg;
    uint8_t buf[512];

    for(;;) {
        int n = uart_read_bytes(c->uart, buf, sizeof(buf), pdMS_TO_TICKS(2));
        if(n <= 0)
            continue;

        if(pass == c - ch) {
            uart_write_bytes(CONSOLE_UART, buf, n);
            continue;
        }

        xSemaphoreTake(c->lock, portMAX_DELAY);
        if(c->dev && cdc_acm_host_data_tx_blocking(c->dev, buf, n, USB_TX_MS) == ESP_OK) {
            c->to_disp += n;
            activity = xTaskGetTickCount();
        }
        xSemaphoreGive(c->lock);

        if(!usb_plugged())
            net_send(c, buf, n);
    }
}

static void uart_start (channel_t *c)
{
    const uart_config_t cfg = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(c->uart, UART_BUF, UART_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(c->uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(c->uart, c->tx, c->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_pullup_en(c->rx);          // no noise while the cable is off

    c->lock = xSemaphoreCreateMutex();
    xTaskCreate(uart_task, c->name, 4096, c, 12, NULL);
}

static void usb_lib_task (void *arg)
{
    for(;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if(flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }
}

static bool usb_open (channel_t *c)
{
    const cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = USB_BUF,
        .in_buffer_size = USB_BUF,
        .event_cb = usb_event,
        .data_cb = usb_rx,
        .user_arg = c,
    };
    cdc_acm_dev_hdl_t dev = NULL;

    if(cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, c->iface, &cfg, &dev) != ESP_OK)
        return false;

    cdc_acm_host_set_control_line_state(dev, true, false);  // DTR: the display sends only to an open port

    xSemaphoreTake(c->lock, portMAX_DELAY);
    c->dev = dev;
    xSemaphoreGive(c->lock);

    return true;
}

// The name and password of the machine network to the display, as a line on the link channel:
//   $BW,<ssid>,<password>*<hh>    (both without ',' and '*', checksum as in the display link)
static void send_pairing (void)
{
    char body[80], line[96];
    uint8_t sum = 0;

    snprintf(body, sizeof(body), "BW,%s,%s", AP_SSID, ap_pass);
    for(const char *p = body; *p; p++)
        sum ^= (uint8_t)*p;
    int n = snprintf(line, sizeof(line), "\n$%s*%02X\n", body, sum);   // leading \n: ends a line cut short

    xSemaphoreTake(ch[0].lock, portMAX_DELAY);
    if(ch[0].dev)
        cdc_acm_host_data_tx_blocking(ch[0].dev, (const uint8_t *)line, n, USB_TX_MS);
    xSemaphoreGive(ch[0].lock);
}

// The access point is off while the display is plugged in
static void ap_switch (bool on)
{
    static bool ap_on = true;

    if(on == ap_on)
        return;
    ap_on = on;
    if(on)
        esp_wifi_start();
    else {
        tcp_close();
        esp_wifi_stop();
    }
    ESP_LOGI(TAG, "WiFi access point %s", on ? "on" : "off");
}

// Waits for the display, opens both channels and starts over once it is unplugged.
static void usb_run (void)
{
    for(;;) {
        if(!usb_open(&ch[0]))
            continue;               // nothing plugged in, try again
        ESP_LOGI(TAG, "display connected");
        ap_switch(false);
        send_pairing();
        if(!usb_open(&ch[1]))
            ESP_LOGW(TAG, "display has no gamepad channel (interface %u)", ch[1].iface);

        xSemaphoreTake(gone, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(200));     // the other channel goes too
        while(xSemaphoreTake(gone, 0) == pdTRUE);
        ap_switch(true);
    }
}

// Display link over TCP and gamepad over UDP, only while the display is not plugged in.
static void net_task (void *arg)
{
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY) };
    uint8_t buf[512];

    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    addr.sin_port = htons(LINK_PORT);
    bind(srv, (struct sockaddr *)&addr, sizeof(addr));
    listen(srv, 1);

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    addr.sin_port = htons(GAMEPAD_PORT);
    bind(udp_sock, (struct sockaddr *)&addr, sizeof(addr));

    for(;;) {
        fd_set rd;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        int cl = tcp_client, maxfd = srv > udp_sock ? srv : udp_sock;

        FD_ZERO(&rd);
        FD_SET(srv, &rd);
        FD_SET(udp_sock, &rd);
        if(cl >= 0) {
            FD_SET(cl, &rd);
            if(cl > maxfd)
                maxfd = cl;
        }

        if(select(maxfd + 1, &rd, NULL, NULL, &tv) > 0) {

            // a new connection replaces the old one, the display may have lost WiFi in between
            if(FD_ISSET(srv, &rd)) {
                int s = accept(srv, NULL, NULL);
                if(s >= 0) {
                    int on = 1;
                    struct timeval to = { .tv_sec = 0, .tv_usec = NET_TX_MS * 1000 };
                    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
                    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
                    tcp_close();
                    xSemaphoreTake(tcp_lock, portMAX_DELAY);
                    tcp_client = s;
                    tcp_seen = xTaskGetTickCount();
                    xSemaphoreGive(tcp_lock);
                    ESP_LOGI(TAG, "link: display connected over WiFi");
                }
            }

            if(cl >= 0 && cl == tcp_client && FD_ISSET(cl, &rd)) {
                int n = recv(cl, buf, sizeof(buf), 0);
                if(n <= 0)
                    tcp_close();
                else {
                    tcp_seen = xTaskGetTickCount();
                    if(!usb_plugged())
                        forward_to_ctrl(&ch[0], buf, n);
                }
            }

            if(FD_ISSET(udp_sock, &rd)) {
                struct sockaddr_in from;
                socklen_t flen = sizeof(from);
                int n = recvfrom(udp_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);
                if(n > 0 && !usb_plugged()) {
                    udp_peer = from;
                    udp_seen = xTaskGetTickCount();
                    forward_to_ctrl(&ch[1], buf, n);
                }
            }
        }

        // the display sends a heartbeat every 500 ms, silence means it is gone (or now on USB)
        if(tcp_client >= 0 && !recent(tcp_seen))
            tcp_close();
    }
}

// Access point of the machine network, the password is made once and kept in NVS
static void wifi_start (void)
{
    nvs_handle_t nvs;
    size_t len = sizeof(ap_pass);

    if(nvs_flash_init() != ESP_OK) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    if(nvs_open("bridge", NVS_READWRITE, &nvs) == ESP_OK) {
        if(nvs_get_str(nvs, "pass", ap_pass, &len) != ESP_OK || strlen(ap_pass) != AP_PASS_LEN) {
            static const char chars[] = "abcdefghijkmnpqrstuvwxyzABCDEFGHJKLMNPQRSTUVWXYZ23456789";
            for(int i = 0; i < AP_PASS_LEN; i++)
                ap_pass[i] = chars[esp_random() % (sizeof(chars) - 1)];
            ap_pass[AP_PASS_LEN] = '\0';
            nvs_set_str(nvs, "pass", ap_pass);
            nvs_commit(nvs);
        }
        nvs_close(nvs);
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    wifi_config_t cfg = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 1,           // not in any scan list, the display gets it over the USB cable
        },
    };
    strlcpy((char *)cfg.ap.password, ap_pass, sizeof(cfg.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    xTaskCreate(net_task, "net", 6144, NULL, 11, NULL);
}

// The way stays visible, data flowing (the gamepad sends all the time) adds a short green flash a second
static void led_task (void *arg)
{
    TickType_t flash = 0;

    for(;;) {
        uint8_t r = 0, g = 0, b = 0;
        TickType_t now = xTaskGetTickCount();

        if(now - flash >= pdMS_TO_TICKS(1000) && now - activity < pdMS_TO_TICKS(1000))
            flash = now;

        if(pass >= 0)
            r = b = 16;
        else if(!usb_plugged() && tcp_client < 0)
            r = 24;
        else if(now - flash < pdMS_TO_TICKS(LED_DATA_MS))
            g = 32;
        else if(usb_plugged())
            b = 24;
        else
            g = b = 20;

        led_strip_set_pixel(led, 0, r, g, b);
        led_strip_refresh(led);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void led_start (void)
{
    const led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    const led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };

    if(led_strip_new_rmt_device(&cfg, &rmt, &led) == ESP_OK)
        xTaskCreate(led_task, "led", 3072, NULL, 2, NULL);
}

static void status (void)
{
    printf("Octopus-Bridge %s, built %s %s\n", BRIDGE_VERSION, __DATE__, __TIME__);
    for(int i = 0; i < 2; i++)
        printf("  %-8s UART%d TX GPIO%d RX GPIO%d %d baud, display %s, %lu bytes to the controller, %lu to the display\n",
                ch[i].name, ch[i].uart, ch[i].tx, ch[i].rx, BRIDGE_BAUD,
                ch[i].dev ? "connected" : (i == 0 ? tcp_client >= 0 : recent(udp_seen)) ? "over WiFi" : "-",
                (unsigned long)ch[i].to_ctrl, (unsigned long)ch[i].to_disp);

    wifi_sta_list_t sta;
    printf("  WiFi '%s' channel %d", AP_SSID, AP_CHANNEL);
    if(esp_wifi_ap_get_sta_list(&sta) == ESP_OK) {
        printf(", %d device(s)", sta.num);
        for(int i = 0; i < sta.num; i++)
            printf(", %02x:%02x:%02x:%02x:%02x:%02x %d dBm", sta.sta[i].mac[0], sta.sta[i].mac[1], sta.sta[i].mac[2],
                    sta.sta[i].mac[3], sta.sta[i].mac[4], sta.sta[i].mac[5], sta.sta[i].rssi);
    }
    printf("\n");
}

static void command (char *line)
{
    if(!strcmp(line, "status"))
        status();
    else if(!strcmp(line, "wifi"))
        printf("network '%s', password '%s' (the display gets both over the USB cable)\n", AP_SSID, ap_pass);
    else if(!strcmp(line, "link") || !strcmp(line, "gamepad")) {
        pass = strcmp(line, "link") ? 1 : 0;
        printf("pass-through %s at %d baud, display data is dropped meanwhile, Ctrl+] ends it\n", line, BRIDGE_BAUD);
    } else if(*line)
        printf("commands: status, wifi (network and password), link, gamepad (pass-through of that UART to this console)\n");
}

static void console_task (void *arg)
{
    char line[32];
    int len = 0;
    uint8_t buf[64];

    printf("> ");
    for(;;) {
        int n = uart_read_bytes(CONSOLE_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));

        for(int i = 0; i < n; i++) {
            int p = pass;
            if(p >= 0) {
                if(buf[i] == PASS_EXIT) {
                    pass = -1;
                    printf("\npass-through off\n> ");
                } else
                    uart_write_bytes(ch[p].uart, &buf[i], 1);
            } else if(buf[i] == '\r' || buf[i] == '\n') {
                if(len || buf[i] == '\r') {
                    line[len] = '\0';
                    printf("\n");
                    command(line);
                    len = 0;
                    if(pass < 0)
                        printf("> ");
                }
            } else if((buf[i] == 8 || buf[i] == 127) && len) {
                len--;
                printf("\b \b");
            } else if(buf[i] >= ' ' && len < sizeof(line) - 1) {
                line[len++] = buf[i];
                putchar(buf[i]);
            }
        }
        fflush(stdout);
    }
}

void app_main (void)
{
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 1024, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONSOLE_UART);
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\n");
    status();

    gone = xSemaphoreCreateCounting(4, 0);
    tcp_lock = xSemaphoreCreateMutex();     // before the UART tasks, they send over WiFi too
    for(int i = 0; i < 2; i++)
        uart_start(&ch[i]);
    wifi_start();
    led_start();

    const usb_host_config_t host = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 15, NULL);
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

    usb_run();
}
