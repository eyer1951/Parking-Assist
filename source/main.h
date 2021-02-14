

#define BASE_SSID "xxxxxxxxxxxxx"
#define BASE_PASSWORD "xxxxxxxxxxx"

#define MY_CITY "Seattle"
#define MY_TIME_ZONE "America/Los_Angeles"
#define MDNS_HOST "Parkme"

// ------------------------------------------ PARAMETERS -----------------------------------------------
#define NO_OF_PARAMS 14   /* this project */

enum Param {flags, target_distance_10th_in, approach_zone_depth_10th_in, landing_zone_depth_10th_in, 
                garage_door_clearance_10th_in, timeout_after_stable_sec, high_precision_sample_per_msec, 
                background_sample_per_msec, hour_turn_on, hour_turn_off, hysteresis_10th_in, lcd_display_interval_ms, 
                glitch_limit_pct, max_led_brightness_pct, vacancy_delay_sec};

