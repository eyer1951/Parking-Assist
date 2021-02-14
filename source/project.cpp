/*
    project.cpp -- 13 January 2021

    This library provides the following functionality for projects:
    * Connection to a local WiFi access point, automatically retries if connection lost
    * Establishes an MDNS local network address
    * Supports configurability by a set of parameters defined by the project
    * Parameters can be set/changed by a web application based on a JSON config file
    * main.h header file defines parameter names and default values
    * Parameters are saved in NVS, restored on boot
    * Parameters settable via Websocket interface
    * Each parameter is a 32-bit integer, so it can support FastLED HSV colors (359-255-255)
    * parameter[0] is flags
    * Responds to "Q" Websocket command to report all parameters to web app
*/
#include "project.h"
#include "main.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <WebSocketsServer.h>
#include "SPIFFS.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

extern char version[];

#define ESP_MAXIMUM_RETRY 20
#define REATTEMPT_INTERVAL_MS  60 * 5 * 1000
#define HW_RESET_PIN 16

using namespace std;

nvs_handle my_nvs_handle;
static EventGroupHandle_t task_group;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static int s_retry_num = 0;
tcpip_adapter_ip_info_t info;
bool params_changed = false;
Params parameters;
WebSocketsServer webSocket(81);
char base_hostname[30];

#define PLEASE_STOP_MAIN_BIT 1
static int WSopen = 0;  // true/false, is WS connection open?
extern WiFiServer server;

// ---------------------------------------------- NVS ------------------------------------------------
void send_param_set(uint8_t num);
void immediately_set(string s);

void initialize_nvs() {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
//    ESP_LOGI(TAG, "Opening Non-Volatile Storage(NVS) handle");
    ret = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Error [%s] opening NVS handle!", esp_err_to_name(ret));
}

bool save_params() {
    size_t size = sizeof(parameters);
    params_changed = false;
    if (nvs_set_blob(my_nvs_handle, "Params", &parameters, size) != ESP_OK) {
        printf("Error saving parameters\n");
        return false;
    }
    return true;
}

void restore_parameters(Parameter * p, int count) {  // fetch parameters from NVS if available
    Params x;
    esp_err_t ret;
    size_t size = sizeof(Params);
    ret = nvs_get_blob(my_nvs_handle, "Params", &x, &size);
    if (ret == ESP_OK) {
        memcpy(parameters, x, size);
        printf("Restored flags to %u\n", parameters[flags]);
        for (int i = 0; i < NO_OF_PARAMS; i ++) {
            printf(" - %s to %u\n", p[i].readable_name.c_str(), parameters[i + 1]);
        }
   } else {
        printf("Using default parameter values (%i)\n", int(ret));
        printf("flags: 0\n");
        for (int i = 0; i < NO_OF_PARAMS; i ++) {
            printf("Set %s to %u default\n", p[i].readable_name.c_str(),p[i].default_value);
            parameters[i + 1] = p[i].default_value;
        }
        save_params();
    }
}

// ------------------------------------------ Utilities ----------------------------------------------
string int_to_string(int i) {
	string s;
	int r = i % 10;
	s = (char)(r + 48);
	i = (i - r) / 10;
	r = i % 10;
	while (i > 0) {
		s = (char)(r + 48) + s;
		i = (i - r) / 10;
		r = i % 10;
	}
	return s;
}

int str_to_int(string s) {
    int r = 0, m = 1;
    if (!s.length()) return 0;
    for (int i = s.length() - 1; i >=0; i--) {
        r += ((uint8_t)s[i] - 48) * m;
        m *= 10;
    }
    return r;
}

void printHexArray(uint8_t *arr, uint8_t len) {  // for testing;  groups of 4 bytes per line
    for (int i = 0; i < len; i++) {
        printf("%.2X ", arr[i]);
        if (i % 4 == 3) printf("\n");
    }
}

// -------------------------------------------- mDNS Stuff -------------------------------------------
bool start_mdns() {
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return false;
    }

    //set hostname
    mdns_hostname_set(&base_hostname[0]);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_ws", "_tcp", 81, NULL, 0);
    esp_err_t e = mdns_service_add(NULL, "_ping", "_tcp", 22, NULL, 0);
    printf("MDNS Services added for %s (%i)\n", base_hostname, e);
    return true;
}

// ----------------------------------------------- OTA -----------------------------------------------

void hardwareReset() {
    pinMode(HW_RESET_PIN, OUTPUT);
    digitalWrite(HW_RESET_PIN, false);
    delay(2000);
}

static void initialize_OTA() {
    ArduinoOTA.onStart([]() {
        string stype;
        if (ArduinoOTA.getCommand() == U_FLASH)
            stype = "sketch";
        else  // U_SPIFFS
            stype = "filesystem";
        // arrange for main loop to stop
        xEventGroupSetBits(task_group, PLEASE_STOP_MAIN_BIT);
        printf("Starting OTA update of %s\n", stype.c_str());
    });
    ArduinoOTA.onEnd([]() {
        printf("\n\e[0;32mSUCCESS!\e[0m\nHardware reset via GPIO16...\n");  // '\e[0;32m' sets green text mode
        delay(100);  // let the printf finish before self-reset
        hardwareReset();
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        printf("\n");
        printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            printf("Auth Failed\n");
        else if (error == OTA_BEGIN_ERROR)
            printf("Begin Failed\f");
        else if (error == OTA_CONNECT_ERROR)
            printf("Connect Failed\n");
        else if (error == OTA_RECEIVE_ERROR)
            printf("Receive Failed\n");
        else if (error == OTA_END_ERROR)
            printf("End Failed\n");
        xEventGroupClearBits(task_group, PLEASE_STOP_MAIN_BIT);
    });

    ArduinoOTA.begin();
}
// ------------------------------------------ WebSockets --------------------------------------------------
void send_parameters(uint8_t num) {
// num=255 indicates broadcast
    String s3 = "Q[";
    for (int i = 0; i < PARAM_COUNT - 1; i++) {
        s3 += String(parameters[i]) + ",";
    }
    s3 += String(parameters[PARAM_COUNT - 1]) + "]";
    printf("[Q]: %s\n", s3.c_str());
    if (num < 255)
        webSocket.sendTXT(num, s3.c_str());
    else 
        webSocket.broadcastTXT(s3);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) { // When a WebSocket message is received
    String payload_str = String((char*)payload);
    String angle_str, bpm_str, per_str, ip_str, smo_str, ok_str, p_str;
    string s3;
    esp_err_t ret;
    switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
        printf("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED: {          // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        webSocket.sendTXT(num, ip_str);
        delay(100);
        break;
    }
    case WStype_TEXT: {                  // if new text data is received
        printf("[%u] Received: %s\n", num, payload);
        if (payload_str == "eraseNVS") {
            ret = nvs_flash_erase();
            if (ret == ESP_OK) {
                printf("Erased/initialized NVS\n");
                webSocket.sendTXT(num, "e:OK");
                initialize_nvs();
            } else {
                printf("Problem erasing NVS: %d\n", ret);
                webSocket.sendTXT(num, "e:ERROR");
            }
            return;
        }
        if (payload_str == "S?") {
            send_param_set(num);
            return;
        }
        if (payload_str[0] == 'S') {  // received full parameter set
            char *token = strtok((char*)payload, "[");
            for (int j = 0; j < PARAM_COUNT; j ++) {
                token = strtok(NULL, ",");
                int i = String(token).toInt();
//                printf("%s  (%i)\n", token, i);
                parameters[j] = i;
            }
            save_params();
        }
        if (payload_str[0] == 'Q') {  // Query parameters
            send_parameters(num);
            return;
        }
        if (payload_str[0] == 'P') {
            immediately_set(payload_str.c_str());
            save_params();
        }
        if (payload_str[0] == 'I') {  // Immediate set
            immediately_set(payload_str.c_str());
            params_changed = true;
            return;
        }
        if (payload_str[0] == 'B') {  // Binary flags
            p_str = payload_str.substring(2);
            parameters[flags] = p_str.toInt();
            printf("Setting flags to %u\n", parameters[flags]);
            save_params();
        }
        break;
    }
    case WStype_BIN:
        printf("Received %u bytes of binary data\n", length);
        break;
    default:
        return;
    }
}

// --------------------------------------------- WiFi ------------------------------------------------
#define WIFI_DISCONNECT_REASON_AUTH_EXPIRE 2
#define WIFI_DISCONNECT_REASON_NOT_AUTHED 6
#define WIFI_DISCONNECT_REASON_BEACON_TIMEOUT 200
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND 201
#define WIFI_DISCONNECT_REASON_AUTH_FAIL 202
#define WIFI_DISCONNECT_REASON_ASSOC_FAIL 203
#define WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT 204

String showDisconnectReason(int i) {
    switch (i) {
        case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:                return F("Auth expired");
        case WIFI_DISCONNECT_REASON_NOT_AUTHED:                 return F("Not authorized");
        case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:             return F("Beacon timeout");
        case WIFI_DISCONNECT_REASON_NO_AP_FOUND:                return F("AP disappeared");
        case WIFI_DISCONNECT_REASON_AUTH_FAIL:                  return F("Authorization fail");
        case WIFI_DISCONNECT_REASON_ASSOC_FAIL:                 return F("Association fail");
        case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:          return F("Handshake timeout");
  }
  return "Unknown (" + String(i) + ")";
}

bool wifi_connect_now(bool do_disconnect) {
    int tries = 0;
    bool now_connected = false;
    if (do_disconnect) {
        WiFi.persistent(false);
        WiFi.disconnect(true);
        delay(500);
    }

    WiFi.begin(BASE_SSID, BASE_PASSWORD);  // channel 1 specified

    while (!now_connected) {
        // Wait for WiFi connection
        printf(".");
        fflush(stdout);
        delay(100);
        tries++;
        if (tries > ESP_MAXIMUM_RETRY * 2) {
            printf("\nTimeout waiting for %s\n", BASE_SSID);
            return false;
        }
        now_connected = xEventGroupGetBits(s_wifi_event_group) && WIFI_CONNECTED_BIT;
    }
    return true;
}

static void initialize_wifi();  // forward ref
void initialize_telnet();

void WiFiEvent(WiFiEvent_t event, system_event_info_t infor) {
    switch (event) {
        case SYSTEM_EVENT_STA_START:
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num = 0;
            if (!WSopen) {
                tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
                printf("Got IP address %s\n", ip4addr_ntoa(&info.ip));
                printf("MAC: %s\n", WiFi.macAddress().c_str());
                initialize_OTA();
                webSocket.begin();                  // start the websocket server
                webSocket.onEvent(webSocketEvent);  // if there's an incomming websocket message, go to
                                                    // function 'webSocketEvent'
                printf("Started WebSocket server\n");
                initialize_telnet();
	            printf("\n\e[0;32mConnected to %s on channel %i\e[0m\n", BASE_SSID, WiFi.channel());
                WSopen = 1;
                mdns_free();  // Stop and free mDNS server.
                vTaskDelay(200);
                if (start_mdns()) printf("Started mDNS OK\n");
           }
             break;
		case SYSTEM_EVENT_STA_CONNECTED:
			printf("\nStation connected\n");
			break;
        case SYSTEM_EVENT_STA_DISCONNECTED: {
            if (xEventGroupGetBits(task_group) && PLEASE_STOP_MAIN_BIT) {
                printf("Disconnected after OTA\n");
                break;
            };
            int reason = infor.disconnected.reason;
            if (reason == WIFI_DISCONNECT_REASON_AUTH_FAIL) {
                printf("~~");
                delay(500);
                WiFi.begin(BASE_SSID, BASE_PASSWORD);
                delay(100);
                break;
            }
            if (reason == WIFI_DISCONNECT_REASON_AUTH_EXPIRE) {
                WiFi.persistent( false );
                WiFi.setAutoConnect(false);
            }
             printf("STA disconnected (%i)  Reason (%s)\n", s_retry_num, showDisconnectReason(reason).c_str());
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            WSopen = 0;
            if (s_retry_num < ESP_MAXIMUM_RETRY) {
                printf("Quick retry to connect to %s\n", BASE_SSID);
                wifi_connect_now(true);
                s_retry_num++;
            } else {
                printf("Connect to %s failed. Waiting a while to try again.", BASE_SSID);
                vTaskDelay(REATTEMPT_INTERVAL_MS / portTICK_PERIOD_MS);
                wifi_connect_now(true);
                s_retry_num = 0;
            }
            break;
        }
        default:
            break;
    }
}

static void initialize_wifi() {
    WiFi.mode(WIFI_STA);
	s_wifi_event_group = xEventGroupCreate();
    WiFi.onEvent(WiFiEvent);

    if (!wifi_connect_now(true))  // start by disconnecting 
        printf("Could not connect to %s\n", BASE_SSID);
	delay(1000);
}

void maybe_save_params() {
    if (params_changed) {
        save_params();
        params_changed = false;
        printf("Saved parameters\n");
    }
}

void send_param_set(uint8_t num) {
    // if num==255, broadcast
    // first param is flag set
    String payload = "S:[";
    for (int i = 0; i < PARAM_COUNT - 1; i++) {
        payload += String(parameters[i]) + ", ";
    }
    payload += String(parameters[PARAM_COUNT - 1]) + "]";
    if (num < 255)
        webSocket.sendTXT(num, payload);  // JSON array of decimal numbers
    else
        webSocket.broadcastTXT(payload);
    printf("Sent: %s\n",payload.c_str());
}

void save_action(xTimerHandle pxTimer) {
    xTimerStop(pxTimer, 0);
    if (save_params())
        printf("Saved parameters\n");
}

void immediately_set(string s) {
    int param_no = str_to_int(s.substr(1, 2));
    if (param_no >= PARAM_COUNT)
        printf("Received unknown parameter %i\n", param_no);
    else
        parameters[param_no] = str_to_int(s.substr(4));
}

void initalize_arduino() {
    initArduino();
    Serial.begin(115200);
    printf("\n\n\e[1;33;40m%s\e[0m\n", version);  // Yellow text
}

void initialize_event_groups() {
    s_wifi_event_group = xEventGroupCreate();
    task_group = xEventGroupCreate();
}

void check_nvs_save(esp_err_t ret, string s, int val) {
    char *cstr = new char[s.length() + 1];
    strcpy(cstr, s.c_str());
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ret = nvs_set_i32(my_nvs_handle, cstr, val);
            if (ret != ESP_OK)
                printf("Error saving first %s value!\n", cstr);
            break;
        default:
            printf("Error(%s) reading NVS!\n", esp_err_to_name(ret));
    }
}

void initialize_hw_reset_pin() {
    pinMode(HW_RESET_PIN, INPUT_PULLUP);
}

void initialize_telnet() {
    server.begin();
    server.setNoDelay(true);
}

void initialize_project() {
    initalize_arduino();
    initialize_event_groups();
    initialize_wifi();
    initialize_nvs();
    initialize_hw_reset_pin();
    delay(100);
    if (!SPIFFS.begin(true)) {
        printf("An Error has occurred while mounting SPIFFS\n");
        return;
    }
}