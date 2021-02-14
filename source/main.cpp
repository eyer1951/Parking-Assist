/*Name:		Park-the-car.cpp
Created:	2/5/2021  1:40PM

STATUS:
    * Connects to home network, reconnects daily after network returns
    * API to control color of LED(s) 
      * (blink on startup to indicate code version)
    * Synchronizes to local time of day (including DST)
    * OTA upload 
    * Temperature, humidity, and distance sensor working
    * Turn system on in the morning and off at night based on settable parameters
    * Lower the sample rate to once every N msec. whenever distance is 
        * greater than threshold or less than threshold
    * Control color of LED (green to red) based on threshold values
    *   Or: number of LEDs lit in a strip
    * Blink red if beyond the "too close" threshold
    * Control of hysteresis
    * Control of brightness level when Homing
    * Supports Telnet connection, "D" command to report last 48 distance measurements
    * Supports mDNS, with "Parkme.local" (or "Parkme2.local") as host ID
*/
char version[] = "Park v1.14";

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ezTime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <math.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "nvs.h"
#include "nvs_flash.h"

#define FASTLED_INTERNAL
#include <FastLED.h>
#include <Adafruit_Sensor.h>    // https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>                // https://github.com/adafruit/DHT-sensor-library
#include <LiquidCrystal_I2C.h>  // Library for LCD

#include "main.h"
#include "project.h"

//------------------------------------------ Options -------------------------------------------------
#define ENABLE_TIME_OF_DAY
// #define ENABLE_LCD

//----------------------------------------- GPIO Assignments -----------------------------------------
#define DHTPin 4        // I/O Pin for DHT11 Humiture unit 
#define DHTPin_ALT 26   // Alternate DHT11 pin (if GPIO4 doesn't work for all ESP32s)
#define DHTType DHT11   // Define DHT sensor type

#define trigPin 2       // I/O Pins for HC-SR04 Ultrasonic Distance Unit
#define echoPin 5

#define BUTTON_PIN 13   // Pushbutton is on GPIO13

#ifdef ENABLE_LCD
// Connect to LCD via i2c, default address 0x27 (A0-A2 not jumpered):
// LCD uses default i2c pins:  SCLPin = 22  SDAPin = 21
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
#endif

//------------------------------------------- DEBUG Items --------------------------------------------
#define CORE_DEBUG_LEVEL ESP_LOG_ERROR
#define GPIO_DEBUG
#define DEBUG_TICKS 1 /* How many cycles debug_time > 0 */
#define DEBUG_INTERVAL_MS 500
volatile int debug_time = 0;

// ------------------------------------------- LED STUFF ---------------------------------------------
#define NUM_LEDS 16
#define NUM_STRANDS 1 

#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define BRIGHTNESS 128
#define MAX_BRIGHTNESS 255
#define MY_CORRECTION 0xFEC886 /* set by testing */
#define RMT_CH0_GPIO 19
#define MAX_LED_BRIGHTNESS parameters[max_led_brightness_pct] * 255 / 100
static CRGB leds[NUM_STRANDS][NUM_LEDS + 1];  // the +1 holds overflow (not visible)

//---------------------------------------------- Globals ---------------------------------------------
using namespace std;

#define PRESSED LOW
#define NOT_PRESSED HIGH

typedef struct Buttons {
    const byte pin = BUTTON_PIN;
    const int debounce = 10;
 
    unsigned long counter=0;
    bool prevState = NOT_PRESSED;
    bool currentState;
} Button;
 
// create a Button variable type
Button button;

long duration;
float distance_cm;
float distance_in;
float distance_ft;
float speedofsound;
float parked_at;
float state_distance;

int temperature;
unsigned long msnow = millis();
unsigned long home_time_ms;

extern nvs_handle my_nvs_handle;
extern Params parameters;
extern WebSocketsServer webSocket;
extern char base_hostname[30];

const char *base_ssid = BASE_SSID;
const char *base_password = BASE_PASSWORD;

#ifdef ENABLE_TIME_OF_DAY
Timezone myTZ;
int hour_now = myTZ.hour();
int minute_now = myTZ.minute();
int second_now = myTZ.second();
#endif

// Queue of distance measurements, for smoothing
int dist_iptr = 0;
int dist_optr = 0;
#define DIST_Q_DEPTH 100
float distances[DIST_Q_DEPTH];

// Blink warning
#define BLINK_INTERVAL 300  // milliseconds
bool blink;

typedef Ring<float> Ringf;
typedef Ring<long> Ringi;

Ringf samples_f(DIST_Q_DEPTH);
Ringi samples_i(DIST_Q_DEPTH);
int skipped = 0;
int skipped_total = 0;

//------------------------------------------ Parameters ----------------------------------------------
Parameter params[NO_OF_PARAMS] = {{"target distance (tenth in.)", 450},
                                  {"approach zone depth (tenth in.)", 480},
                                  {"landing zone depth (tenth in.)", 50},
                                  {"garage door clearance (tenth in.)", 100},
                                  {"timeout after stable (sec.)", 600},
                                  {"high precision sample rate (msec.)", 200},
                                  {"background sample rate (msec.)", 1500},
                                  {"time turn on (minutes since midnight)", 70},
                                  {"time turn off (minutes since midnight)", 230},
                                  {"hysteresis (tenth in.)", 15},
                                  {"lcd display interval (msec.)", 300},
                                  {"glitch limit (pct. x10)", 50},
                                  {"brightness (pct.)", 100},
                                  {"vacancy delay (sec.)", 60}};

//------------------------------------------ State Machine -------------------------------------------
enum State {Night, Vacant, Homing, Home, TooClose, Parked};
State state = Night;

unsigned long display_interval, measurement_interval;
SemaphoreHandle_t xSemaphore_d = NULL;   // distance
SemaphoreHandle_t xSemaphore_t = NULL;   // temperature

#define TEMP_MEASUREMENT_INTERVAL_MS  1000 * 60 /*  1000 * 60 * 5 5 minutes */

#define LONG_TIME 0xFFFFFF

#define UNIT_NO_PIN 27  /* Pulldown when not the one in the garage; for testing */

extern WiFiClient serverClient;
extern tcpip_adapter_ip_info_t info;

DHT dht1(DHTPin, DHTType);
DHT dht2(DHTPin_ALT, DHTType);
DHT *dhtp;
int unit_no;

//-------------------------------------------- LCD Stuff ---------------------------------------------
#ifdef ENABLE_LCD
SemaphoreHandle_t xSemaphore_l = NULL;   // LCD
#define LCD_BUFFSIZE 20
char buffer1[LCD_BUFFSIZE];  // LCD line 1
char buffer2[LCD_BUFFSIZE];  // LCD line 2
#define LCD_DISPLAY_INTERVAL parameters[lcd_display_interval_ms]

void display_off_on_lcd() {
    memset(buffer1, 0, sizeof(buffer1));
    strcpy(buffer1, "off");
    memset(buffer2, 0, sizeof(buffer2));
}
#endif

//------------------------------------------- Telnet -------------------------------------------------
// Telnet
WiFiServer server(23);
WiFiClient serverClient;

// ------------------------------------------ Telnet Print -------------------------------------------
template<typename... Args> void telnet_printf(const char * f, Args... args) {
    char cbuff [360];
    int len;
    if (serverClient || serverClient.connected()) {
        len = sprintf(cbuff, f, args...);
        if (cbuff[len-1] == '\n') cbuff[len++] = '\r';
        serverClient.write(cbuff, len); 
    }
}

template<typename... Args> void tprintf(const char * f, Args... args) {
    printf(f, args...);
    telnet_printf(f, args...);
    telnet_printf("\r");
}

void handle_telnet_command(char c) {
    float items[51], a;
    string results = "";
    char one_item[30];
    int i, n;
    switch (c) {
        case 'D':
        case 'd':
            n = samples_f.get_last_n(49, items);
            for (i = 0; i < 49; i ++) {
                if (i < n) {
                    sprintf(one_item, "%.1f, ", items[i]);
                    results += one_item;
                }
            }
            a = samples_f.average(4);
            telnet_printf("%s (%.1f)\n\r", results.c_str(), a);
            break;
        default:
            break;
    }
}

void new_telnet_client() {
    if (WiFi.status() == WL_CONNECTED) {
        if (server.hasClient()) {
            if (!serverClient || !serverClient.connected()) {
                if (serverClient) serverClient.stop();
                serverClient = server.available();
                if (!serverClient) printf("available broken\n");
                telnet_printf("New telnet client: %s\n\r", serverClient.remoteIP().toString().c_str());
                telnet_printf("\n\e[0;32mConnected to %s on channel %i\e[0m\n\r", BASE_SSID, WiFi.channel());
                tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
                telnet_printf("Got IP address %s\n\r", ip4addr_ntoa(&info.ip));
            }
        }
        if (serverClient && !serverClient.connected()) {
            serverClient.stop();
        }
    }
}

void check_telnet_data() {
    char cbuff[10];
    if (serverClient && serverClient.connected()) {
        if (serverClient.available()) {
            while (serverClient.available()) {
                cbuff[0] = serverClient.read();
                handle_telnet_command(cbuff[0]);
            }
        }
    }
}

//------------------------------------------ Button Handler ------------------------------------------
void handle_button_press() {
    float where_we_are = samples_f.average(samples_f.fullness()-1);
    parameters[target_distance_10th_in] = (int)(0.5 + where_we_are * 10.0);
    tprintf("New target distance set to %.1f inches (%i)\n", where_we_are, parameters[target_distance_10th_in]);
    send_parameters(255);
    save_params();
}

void check_button() {
    button.currentState = digitalRead(button.pin);    // check the button
    if (button.currentState != button.prevState) {    // has it changed?
        delay(button.debounce); 
        button.currentState = digitalRead(button.pin);  // update status in case of bounce
        if (button.currentState == PRESSED) { // has a new press event occured?
            handle_button_press();
        }
        button.prevState = button.currentState; // used to detect when state changes
    } 
}

//------------------------------------------ Time of Day ---------------------------------------------
#ifdef ENABLE_TIME_OF_DAY
void wait_for_local_time() {
	//setDebug(INFO);
	waitForSync();
	printf("UTC:         %s\n", UTC.dateTime().c_str());
	// https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
	myTZ.setLocation(F(MY_TIME_ZONE));
	printf(MY_CITY ":     %s\n", myTZ.dateTime().c_str());
}
#endif

bool during_the_day() {
#ifndef ENABLE_TIME_OF_DAY
    return true;
#else
    int hour_now = myTZ.hour();
    int minute_now = myTZ.minute();
    int since_midnight = (hour_now * 60) + minute_now;
    bool too_early = since_midnight < parameters[hour_turn_on];
    bool too_late = since_midnight > parameters[hour_turn_off];
    return !too_early && !too_late;
#endif
}

// ---------------------------------------- DISPLAY STUFF --------------------------------------------
void clear_leds() { memset(leds, 0, sizeof(leds)); }

void show_version() {
    // Blink the LED corresponding to version LSB
    int z[10] = {0, 6, 2, 8, 4, 1, 7, 3, 9, 5};  // sequence colors semi-randomly
    int i, n, v, h, d;
    CHSV c;
    n = strlen(version);
    v = version[n - 1] - '0';
    h = 255 * z[v] / 10;
    c = CHSV(h, 255, 255);
    if (v > 0)
        d = 150;
    else {
        d = 50;  // LSB zero -> short blink hue 0
        v++;
    }
    for (i = 0; i < v; i++) {
        leds[0][0] = c;
        FastLED.show();
        delay(d);
        leds[0][0] = CRGB::Black;  
        FastLED.show();
        delay(300);
    }
}

void flash_a_light() {
        leds[0][1] = CRGB::White;
    FastLED.show();
    delay(20);
    leds[0][1] = CRGB::Black;
    FastLED.show();
}

void set_all_leds(CRGB c) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[0][i] = c;
    }
    FastLED.show();
}

#define TARGET parameters[target_distance_10th_in] / 10.0
#define AZ_DEPTH parameters[approach_zone_depth_10th_in] / 10.0
#define WARN (parameters[target_distance_10th_in] - parameters[landing_zone_depth_10th_in]) / 10.0
#define GARAGE (parameters[target_distance_10th_in] + parameters[garage_door_clearance_10th_in]) / 10.0
#define APPROACH (parameters[target_distance_10th_in] + parameters[approach_zone_depth_10th_in]) / 10.0
#define HYSTERESIS parameters[hysteresis_10th_in] / 10.0

void show_homing_leds() {
// light top LED white
    float d = samples_f.average(5);
    uint8_t h;
    int max_leds = NUM_LEDS - 1;
    float home = TARGET + 0.5 * AZ_DEPTH;
    clear_leds();
    if (d < WARN) return;
    if (d > home) {
        leds[0][max_leds] = CRGB::White;
        FastLED.show();
        return;
    } 
    // if closer than warning distance, light some other LEDs
    float num_lit_f = max_leds - max_leds * (d - TARGET) / (home - TARGET);  // ok
    int num_lit_i = (int)num_lit_f;
    // compute fractional value for LED one beyond the last one fully lit
    float frac_lit = num_lit_f - num_lit_i;
    uint8_t value = frac_lit * MAX_LED_BRIGHTNESS;
    h = 96 - (36 * num_lit_i / max_leds);  // hue goes from green to yellow as we get closer
    for (int i = 0; i < num_lit_i; i++) {
        leds[0][i] = CHSV(h, 255, MAX_LED_BRIGHTNESS);
    }
    // determine a color/value for fractional LED
    CHSV frac = CHSV(h, 255, value);
    CRGB colo = frac;
    if (colo.green < 2 && colo.blue == 0) colo.red = 0;  // no red-only display
    leds[0][num_lit_i] = colo;
    leds[0][max_leds] = CRGB::White;
    if (d > GARAGE)
        leds[0][0] = CRGB::Red;
    FastLED.show();
    vTaskDelay(10);
}

// ----------------------------------------- SPEED OF SOUND ------------------------------------------
void derive_speed_of_sound() {
    int temp_temp;
    temp_temp = dhtp->readTemperature();
    if (temp_temp < 40 && temp_temp > 10)
        temperature = temp_temp;
    else
        return;
    // Calculate speed of sound in m/s:
    speedofsound = 331.3 + (0.606 * temperature);
}

// --------------------------------------------- DISTANCE --------------------------------------------
void measure_distance() {
    float delta_pct;
    long diff;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    // Trigger the sensor by setting the trigPin high for 10 microseconds:
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echoPin. This returns the duration (length of the pulse) in microseconds:
    long d = pulseIn(echoPin, HIGH);
    if (samples_i.fullness() >=3) {
        long ave = samples_i.average(3);
        if (d > ave) {
            diff = d - ave;
        } else {
            diff = ave - d;
        }
        delta_pct = 1000.0 * (float)diff / (float)ave;  // percent would be x100, but parameter is 10x percent, so x1000
        // add this sample to the queue IF 
        //  * we skipped the previous one, OR
        //  * the sample does not differ from the average by an amount larger than "glitch limit", OR
        //  * we are in Vacant state 
        if (skipped || delta_pct < parameters[glitch_limit_pct] || state == Vacant) {
            samples_i.add(d);
            skipped = 0;
        } else {
            skipped ++;
            skipped_total ++;
        }
    } else {
        samples_i.add(d);
        skipped = 0;
    }
    duration = samples_i.average(3);
    // Calculate the distance in cm:
    distance_cm = duration * (speedofsound / 10000) / 2;
    distance_in = distance_cm / 2.54;
    distance_ft = distance_in / 12.0;
    samples_f.add(distance_in);
}

//------------------------------------------------ Initialization ------------------------------------
static void initialize_humiture() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    if (unit_no == 0)
        dhtp = &dht2;
    else
        dhtp = &dht1;
    dhtp->begin();
#ifdef ENABLE_LCD
    // Initiate the LCD:
    lcd.init();
    lcd.backlight();
#endif
}

void gpioSetup(int gpioNum, int gpioMode, int gpioVal) {
#if defined(ARDUINO) && ARDUINO >= 100
    pinMode(gpioNum, gpioMode);
    digitalWrite(gpioNum, gpioVal);
#endif
}

// ------------------------------------------ Setup Tasks --------------------------------------------
void per_second_task(void *pvParameters) {
    while (1) {
        maybe_save_params();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void temperature_task(void *pvParameters) {
    xSemaphore_t = xSemaphoreCreateBinary();
    while (1) {
        if( xSemaphoreTake( xSemaphore_t, LONG_TIME ) == pdTRUE )    
            if (state != Night) {
                    derive_speed_of_sound();
                }
       vTaskDelay(10);
    }
}

void distance_task(void *pvParameters) {
    xSemaphore_d = xSemaphoreCreateBinary();
    while (1) {
        if( xSemaphoreTake( xSemaphore_d, LONG_TIME ) == pdTRUE )    
            measure_distance();
    }
}

#ifdef ENABLE_LCD
void show_values_on_lcd_task(void *pvParameters) {
    xSemaphore_l = xSemaphoreCreateMutex();
    while (1) {
        if (xSemaphoreTake( xSemaphore_l, LONG_TIME) == pdTRUE) {
            lcd.setCursor(0, 0);
            vTaskDelay(2);
            lcd.printstr(buffer1);
            vTaskDelay(2);
            lcd.setCursor(0, 1);
            vTaskDelay(2);
            lcd.printstr(buffer2);  
            vTaskDelay(2);
//            xSemaphoreGive( xSemaphore_l);
        }
    }
}
#endif

void blink_task(void *pvParameters) {
    while (1) {
        vTaskDelay(BLINK_INTERVAL);
        blink = !blink;
    }
}

void main_loop(void *pvParameters);

void initialize_tasks() {
    xTaskCreatePinnedToCore(per_second_task, "per_second_task", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(temperature_task, "temperature_task", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(blink_task, "blink_task", 2048, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(distance_task, "distance_task", 4096, NULL, 10, NULL, 1);
#ifdef ENABLE_LCD
    xTaskCreatePinnedToCore(show_values_on_lcd_task, "LCD", 4096, NULL, 10, NULL, 1);
#endif
    xTaskCreatePinnedToCore(main_loop, "main", 4096, NULL, 9,  NULL, 1);
}

void initialize_gpio() {
    pinMode(UNIT_NO_PIN, INPUT_PULLUP);
    pinMode(button.pin, INPUT_PULLUP);
}

void initialize_FastLED() {
    clear_leds();
    FastLED.addLeds<CHIPSET, RMT_CH0_GPIO, COLOR_ORDER>(leds[0], NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(MAX_BRIGHTNESS);
    FastLED.setMaxRefreshRate(100);
    FastLED.setDither(0);
}

// --------------------------------------------- Transition State ------------------------------------
// {Night, Vacant, Homing, Home, TooClose, Parked}
void go_Night() {
    tprintf("to Night\n");
    display_interval = 1000;  // per second
    measurement_interval = 0;  // no measurements
#ifdef ENABLE_LCD
    display_off_on_lcd();
#endif
    state = Night;
}

void go_Vacant() {
    display_interval = 100;  // 10 per second
    measurement_interval = parameters[background_sample_per_msec];
    state = Vacant;
}

void go_Homing() {
    display_interval = 20;  // 50 displays per second
    measurement_interval = parameters[high_precision_sample_per_msec];
    state = Homing;
}

void go_Home() {
    home_time_ms = millis();
    display_interval = 20;  // 50 displays per second
    measurement_interval = parameters[high_precision_sample_per_msec];
    state = Home;
}

void go_TooClose() {
    state = TooClose;
    display_interval = 100;  // 10 per second
    measurement_interval = parameters[high_precision_sample_per_msec];
}

void go_Parked() {
    display_interval = 100;  // 10 per second
    measurement_interval = parameters[background_sample_per_msec];
    state = Parked;
    parked_at = samples_f.average(3);
}

void initialize_base_hostname() {
    strcpy(base_hostname,MDNS_HOST);
    if (unit_no == 0)  // Second (test) unit? 
        strcat(base_hostname, "2");
//    strcat(base_hostname, ".local");
}

// ---------------------------------------------- SETUP ----------------------------------------------
void setup() {
    initialize_gpio();
    unit_no = gpio_get_level((gpio_num_t)UNIT_NO_PIN);
    initialize_base_hostname();
    initialize_FastLED();
    // leds[0][NUM_LEDS - 1] = CRGB{5, 0, 5};   // for debug: in case of some early crach
    // FastLED.show();
    initialize_project();
    initialize_humiture();

    printf("Showing version...\n");
    show_version();
    printf("Restoring parameters...\n");
    restore_parameters(params, NO_OF_PARAMS);
#ifdef ENABLE_TIME_OF_DAY
    printf("Getting time of day...\n");
    wait_for_local_time();
#endif
    printf("High precision sample period (msec.): %i\n", parameters[high_precision_sample_per_msec]);
    derive_speed_of_sound();
    go_Night();

    printf("initializing tasks...\n");
    initialize_tasks();
    samples_f.reset();
}

// ---------------------------------------------- IDLE -----------------------------------------------
void idle(int idle_ms) {
    uint32_t now;
    now = millis();
    while (millis() - now < idle_ms) {
        ArduinoOTA.handle();
        webSocket.loop();
        new_telnet_client();
        check_telnet_data();
        vTaskDelay(1);
        check_button();
    }
}

// -------------------------------------------- MAIN LOOP --------------------------------------------
void main_loop(void *pvParameters) {
    unsigned long measurement_time = millis();
    unsigned long display_time = millis();
    unsigned long temp_time = millis() + TEMP_MEASUREMENT_INTERVAL_MS;
    unsigned long ok_goto_vacancy = 0;
    float homing_limit;
#ifdef ENABLE_LCD
    unsigned long lcd_time = millis();
#endif
    State old_state = Night;

    while (1) {
        float d = samples_f.average(3);
        homing_limit = APPROACH;
        switch (state) {  // {Night, Vacant, Homing, Home, TooClose, Parked}
            case (Night): {
                if (during_the_day()) {
                    state_distance = d;
                    go_Vacant();
                };
                break;
            }
            case (Vacant): {
                if (d < homing_limit) {
                    state_distance = d;
                    go_Homing();
                }
                if (distance_in < (24.0 + APPROACH)) {
                    measurement_interval = parameters[high_precision_sample_per_msec];
                }
                break;
            }
            case (Homing): {
                if (d > (homing_limit + HYSTERESIS)) {
                    state_distance = d;
                    go_Vacant();
                    break;
                }
                if (d <= TARGET) {
                    state_distance = d;
                    go_Home();
                }
                break;
            }
            case (Home): {
                if (d > (HYSTERESIS + TARGET)) {
                    state_distance = d;
                    go_Homing();
                    break;
                }
                if (d <= WARN) {
                    state_distance = d;
                    go_TooClose();
                    break;
                }
                if (millis() - home_time_ms > parameters[timeout_after_stable_sec]) {
                    state_distance = d;
                    ok_goto_vacancy = millis();  // restart millisecond timeout
                    go_Parked();
                }
                break;
            }
            case (TooClose): {
                if (d > (HYSTERESIS + WARN)) {
                    state_distance = d;
                    go_Home();
                    break;
                }
                break;
            }
            case (Parked): {
                if (d < APPROACH) {  // car is still parked here
                    ok_goto_vacancy = millis();  // restart millisecond timeout
                } else { // apparently vacant now; for how long?
                    if ((millis() - ok_goto_vacancy) > (parameters[vacancy_delay_sec] * 1000)) { // really gone
                        state_distance = d;
                        go_Vacant();
                    }
                }
                if (during_the_day()) {
                    idle(500);
                } else {
                    state_distance = d;
                    go_Night();
                }
                break;
            }
        }  // /switch
        // time to measure distance?
        // xSemaphoreTake(xSemaphore_l, 100 );
        if (measurement_interval)  // zero means we're not measuring now
            if (millis() - measurement_time > measurement_interval) {
                measurement_time = millis();
                xSemaphoreGive( xSemaphore_d);
                vTaskDelay(5);
            }

        // time to show the LEDs?
        if (millis() - display_time > display_interval) {
            display_time = millis();
            switch (state) { // {Night, Vacant, Homing, Home, TooClose, Parked}
                case (Night):
                    if (state != old_state) {
                        tprintf("to Night     (%.1f)\n", state_distance);
                        clear_leds();
                        leds[0][0] = CRGB{0, 0, 20};
                        FastLED.show();
                    }
                    break;
                case (Vacant):
                    if (state != old_state) {
                        tprintf("to Vacant    (%.1f)\n", state_distance);
                        clear_leds();
                        leds[0][0] = CRGB{5, 5, 0};  // light yellow, one LED
                        FastLED.show();
                    }
                    break;
                case (Homing):
                    if (state != old_state) {
                        tprintf("to Homing    (%.1f)\n", state_distance);
                    }
                    show_homing_leds();
                    break;
                case (Home):
                    if (state != old_state) {
                        tprintf("to Home      (%.1f)\n", state_distance);
                        set_all_leds(CRGB::Green);
                        FastLED.show();
                    }
                    break;
                case (TooClose):
                    if (state != old_state) { 
                        tprintf("to TooClose  (%.1f)\n", state_distance);
                    }
                    set_all_leds(blink ? CRGB::Red : CRGB::Black);
                    break;
                case (Parked):
                    if (state != old_state) {
                        tprintf("to Parked    (%.1f)\n", state_distance);
                        clear_leds();
                        leds[0][0] = CRGB{0, 5, 0};  // light green, one LED
                        FastLED.show();
                    }
            }
            old_state = state;
        }
        if ( (state > Night) && (millis() - temp_time > TEMP_MEASUREMENT_INTERVAL_MS)) {
            xSemaphoreGive(xSemaphore_t);
            temp_time = millis();
            int hour_now = 0;
            int minute_now = 0;
            int second_now = 0;
#ifdef ENABLE_TIME_OF_DAY
            hour_now = myTZ.hour();
            minute_now = myTZ.minute();
            second_now = myTZ.second();
#endif
            tprintf("Speed of sound: %.1f m/sec.   Temperature = %i  (%02i:%02i:%02i)[%i]\n", speedofsound, temperature, hour_now, minute_now, second_now, skipped_total);
            // reset skipped total
            skipped_total = 0;
        }
#ifdef ENABLE_LCD
        if ( (state > Night) && (millis() - lcd_time > LCD_DISPLAY_INTERVAL)) {
            lcd_time = millis();
            if (distance_cm < 1000.0) {
                memset(buffer1,0, sizeof(buffer1));
                snprintf(buffer1, LCD_BUFFSIZE, "Dist: %5.1f cm", distance_cm);
                memset(buffer2,0, sizeof(buffer2));
                snprintf(buffer2, LCD_BUFFSIZE, "%4.1f in %5.2f ft", distance_in, distance_ft);
                vTaskDelay(2);
                if (strlen(buffer1) > 16) printf("BUFFER1 length: %u\n", strlen(buffer1));
                if (strlen(buffer2) > 16) printf("BUFFER1 length: %u\n", strlen(buffer2));
               xSemaphoreGive( xSemaphore_l);
            }
        }
#endif
        idle(10);
    } // /forever loop
}

void loop() {
    for (;;)
    ;
}