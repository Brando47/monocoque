#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "e36cluster.h"

// define pins
#define UART_TXD 1
#define UART_RXD 3
#define PIN_TACHO 32
#define PIN_SPEEDO 33
#define PIN_FUEL 25
#define PIN_TEMP 26
#define PIN_ECON 27

#define WATCH_UART_STACK_SIZE 4096
const char start_byte = 0xAA;

// gauge calibrations
// inputs to map() e.g.{rpm.low, rpm.high, out_tacho.low, out_tacho.high}
struct gauge_cal_points {
    const float outLo, outHi, inLo, inHi;
};
struct gauge_cal_points rpm_tacho_map = {500, 7000, 36, 247};
struct gauge_cal_points speed_speedo_map = {20, 220, 27, 285}; // should be able to work out a basic multiplication factor based on 9 teeth on wheel speed reluctor ring and whatever the factory tyre size (wheel diameter) is
// struct gauge_cal_points fuel_fuel_map = {0, 60, 10, 73.5}; // tune to extremeties (inaccurate in the middle)
struct gauge_cal_points fuel_fuel_map = {15, 45, 114, 242}; // balanced tune for best accuracy (slightly underreads at 0, slightly overreads at 60)
struct gauge_cal_points temp_map = {-20, 125, 1024, 123};
struct gauge_cal_points cons_econ_map = {0, 255, 255, 0};

// gauge limits (used with constrain() for raw outputs)
struct gauge_limit {
    const float min, max;
};
struct gauge_limit tacho_limit = {16, 310}; // tone frequency
struct gauge_limit speedo_limit = {16, 400}; // tone frequency
struct gauge_limit fuel_limit = {1, 300}; // PWM duty cycle
struct gauge_limit temp_limit = {80, 1023}; // PWM duty cycle
struct gauge_limit econ_limit = {1, 255}; // PWM duty cycle

// parameters for gauge sweep
struct gauge_limit tacho_sweep_range = {250, 7000}; // rpm
struct gauge_limit speedo_sweep_range = {20, 220}; // kmh

// other constants
#define ALT_ECON_GAUGE_VAR  0 // e.g. set to "15" for testing or "turboboost" for boost gauge, "(gas / 8.5)" for throttle, etc., output should be a number from 0-30 (to match the gauge). Use 0 to keep default econ gauge
const float out_econ_alpha = 0.1; // smoothing factor for economy gauge output
#define E36_INJECTOR_SIZE   21.5 * 10.5 / 60000 // 21.5lb/hr -> cc/min -> L/s
#define E36_INJECTORS       4 // number of injectors
#define ECON_DIVIDER        2. // e.g. halve the tacho signal and the fuel flow rate, allows full use of econ gauge at high speeds (where normally injectors would exceed 100% DC)
#define INJECTOR_PULSE_RATE ECON_DIVIDER / 2.
const float max_fuel_flow = E36_INJECTORS * E36_INJECTOR_SIZE * ECON_DIVIDER;

float temp_Beta;

// handy functions
float map(float input, struct gauge_cal_points c) {
    return (input - c.outLo) * (c.inHi - c.inLo) / (c.outHi - c.outLo) + c.inLo;
}

float constrain(float input, struct gauge_limit l) {
    return input < l.min ? l.min : (input > l.max ? l.max : input);
}

float cal(float input, struct gauge_cal_points c, struct gauge_limit l) {
    return constrain( map(input, c), l);
}

float calc_temp_beta(struct gauge_cal_points c) {
    return log(c.inLo / c.inHi) / ( 1/(c.outLo + 273.15) - 1/(c.outHi+273.15) );
}

float cal_temp(float t, float beta, struct gauge_cal_points c) {
    return c.inHi * pow(2.7183, beta * (1/(t + 273.15) - 1/(c.outHi + 273.15)));
}

float update_ema(float new_val, float current_ema, float alpha) {
    // exponential moving average
    return alpha * new_val + (1 - alpha) * current_ema;
}

//PWM and tone constants
#define LED_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define TACHO_TIMER LEDC_TIMER_0
#define TACHO_CHAN LEDC_CHANNEL_0
#define SPEEDO_TIMER LEDC_TIMER_1
#define SPEEDO_CHAN LEDC_CHANNEL_1
#define ECON_TIMER LEDC_TIMER_3
#define PWM_TIMER LEDC_TIMER_2 // shared timer for fuel/temp/econ
#define FUEL_CHAN LEDC_CHANNEL_2
#define TEMP_CHAN LEDC_CHANNEL_3
#define ECON_CHAN LEDC_CHANNEL_4
const int pwm_res = 10; //TODO
const int tone_dc = 128;
const int pwm_freq = 5000; // arduino uno uses 500, can comfortably do 5000 on 10bit resolution
const uart_port_t uart_num = UART_NUM_0;
const int uart_buffer_size = (1024 * 2); //TODO: just make BYTE_SIZE ?


#define BYTE_SIZE sizeof(E36ClusterData)
E36ClusterData sd;


// Setup UART
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

char uart_buff[128]; // buffer ready for convenience e.g. sprintf(uart_buff, "%f", var);echo_uart(uart_buff);
void echo_uart (char *output_str) {
    size_t len = strlen(output_str);
    char* data = (char*)malloc((len + 2) * sizeof(char));
    strcpy(data, output_str);
    data[len] = '\n';
    data[len + 1] = '\0';
    uart_write_bytes(uart_num, data, strlen(data));
    free(data);
}

// Setup PWM for outputs
static void ledc_init(void)
{
    // TACHO
    ledc_timer_config_t ledc_tacho_timer = {
        .speed_mode       = LED_SPEED_MODE,
        .duty_resolution  = 10,
        .timer_num        = TACHO_TIMER,
        .freq_hz          = tacho_limit.min,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_tacho_timer));
    ledc_channel_config_t ledc_tacho_channel = {
        .speed_mode     = LED_SPEED_MODE,
        .channel        = TACHO_CHAN,
        .timer_sel      = TACHO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_TACHO,
        .duty           = 512, // 50% DC @10bit
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_tacho_channel));

    // SPEEDO
    ledc_timer_config_t ledc_speedo_timer = {
        .speed_mode       = LED_SPEED_MODE,
        .duty_resolution  = 10,
        .timer_num        = SPEEDO_TIMER,
        .freq_hz          = speedo_limit.min,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_speedo_timer));
    ledc_channel_config_t ledc_speedo_channel = {
        .speed_mode     = LED_SPEED_MODE,
        .channel        = SPEEDO_CHAN,
        .timer_sel      = SPEEDO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_SPEEDO,
        .duty           = 512, // 50% DC @10bit
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_speedo_channel));

    // ECON TONE CONFIG
    ledc_timer_config_t ledc_econ_timer = {
        .speed_mode       = LED_SPEED_MODE,
        .duty_resolution  = 8,
        .timer_num        = ECON_TIMER,
        .freq_hz          = tacho_limit.min,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_econ_timer));
    ledc_channel_config_t ledc_econ_channel = {
        .speed_mode     = LED_SPEED_MODE,
        .channel        = ECON_CHAN,
        .timer_sel      = ECON_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_ECON,
        .duty           = tone_dc, //set to zero to disable output
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_econ_channel));

    // FUEL / TEMP
    ledc_timer_config_t ledc_pwm_timer = {
        .speed_mode       = LED_SPEED_MODE,
        .duty_resolution  = 10,
        .timer_num        = PWM_TIMER,
        .freq_hz          = pwm_freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_pwm_timer));

    // FUEL
    ledc_channel_config_t ledc_fuel_channel = {
        .speed_mode     = LED_SPEED_MODE,
        .channel        = FUEL_CHAN,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_FUEL,
        .duty           = fuel_limit.min,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_fuel_channel));

    // TEMP
    ledc_channel_config_t ledc_temp_channel = {
        .speed_mode     = LED_SPEED_MODE,
        .channel        = TEMP_CHAN,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_TEMP,
        .duty           = temp_limit.max,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_temp_channel));
}

static void gauge_sweep(void) {

    const float sweep_time_ms = 7600;
    const float end_sweep_wait_ms = 400;
    const float fps = 60;
    
    const float hold_ms = 1000. / fps; 
    const float steps = sweep_time_ms / hold_ms / 2.; //for half sweep
    const float step_size = 1. / steps; // i.e. from 0 to 1

    struct gauge_cal_points tacho_sweep_range_cal = {0, 1, tacho_sweep_range.min, tacho_sweep_range.max};
    struct gauge_cal_points speedo_sweep_range_cal = {0, 1, speedo_sweep_range.min, speedo_sweep_range.max};

    ESP_ERROR_CHECK(ledc_set_duty(LED_SPEED_MODE, FUEL_CHAN, fuel_limit.min));
    ESP_ERROR_CHECK(ledc_update_duty(LED_SPEED_MODE, FUEL_CHAN));
    ESP_ERROR_CHECK(ledc_set_duty(LED_SPEED_MODE, TEMP_CHAN, 200)); //temp_limit.max
    ESP_ERROR_CHECK(ledc_update_duty(LED_SPEED_MODE, TEMP_CHAN));
    ESP_ERROR_CHECK(ledc_set_duty(LED_SPEED_MODE, ECON_CHAN, econ_limit.min));
    ESP_ERROR_CHECK(ledc_update_duty(LED_SPEED_MODE, ECON_CHAN));

    int8_t rising = 1;

    for (float step = step_size; step >= 0; step = step + rising * step_size) {
        
        // update tacho
        float rpms = map(step, tacho_sweep_range_cal);
        ledc_set_freq(LED_SPEED_MODE, TACHO_TIMER, cal(rpms, rpm_tacho_map, tacho_limit));
        
        // update speedo
        double speed = map(step, speedo_sweep_range_cal);
        ledc_set_freq(LED_SPEED_MODE, SPEEDO_TIMER, cal(speed, speed_speedo_map, speedo_limit));

        vTaskDelay(pdMS_TO_TICKS(hold_ms));

        if (step >=1 && rising != -1) {
            rising = -1;
            vTaskDelay(pdMS_TO_TICKS(end_sweep_wait_ms));
        }
    }
    ledc_set_freq(LED_SPEED_MODE, TACHO_TIMER, tacho_limit.min);
    ledc_set_freq(LED_SPEED_MODE, SPEEDO_TIMER, speedo_limit.min);
}


static void watch_uart(void *arg) {

    uart_flush(uart_num); // in case ESP32 is booted while being sent serial

    float fuel_last = 0;
    int64_t fuel_time_now = esp_timer_get_time(); // microseconds
    int64_t fuel_time_last = fuel_time_now; // microseconds
    int64_t fuel_time_elapsed = 0; // microseconds
    float fuel_cons = 0;
    float out_econ = 0;
    float out_econ_last = 0;

    float Lp100km = 0; // Litres / 100km (mostly used for testing/calibrating gauge)


    for(;;) {

        // Read data from UART.
        int length = 0;
        char buff[BYTE_SIZE];

        do {
            length = uart_read_bytes(uart_num, buff, BYTE_SIZE, 100 / portTICK_PERIOD_MS); // timeout = 100ms
        } while (buff[0] != start_byte || length < BYTE_SIZE); // wait until a clean read of the buffer
        
        memcpy(&sd, &buff, BYTE_SIZE);

        int gas = sd.gas;
        int rpms = sd.rpms;
        double speed = sd.wheelspeed;
        double fuel = sd.fuel;
        int temp = sd.tyretemp;
        double turboboost = sd.turboboost;

        // update tacho
        int out_tacho = cal(rpms, rpm_tacho_map, tacho_limit);
        // sprintf(uart_buff, "rpms = %i  out_tacho = %i", rpms, out_tacho);echo_uart(uart_buff);
        ledc_set_freq(LED_SPEED_MODE, TACHO_TIMER, out_tacho);

        // update speedo
        int out_speedo = cal(speed, speed_speedo_map, speedo_limit);
        if (speed < 1) {
            ESP_ERROR_CHECK(ledc_timer_pause(LED_SPEED_MODE, SPEEDO_TIMER));
        }
        else {
            ESP_ERROR_CHECK(ledc_timer_resume(LED_SPEED_MODE, SPEEDO_TIMER));
            ledc_set_freq(LED_SPEED_MODE, SPEEDO_TIMER, out_speedo);
        }

        // update econ
        fuel_time_now = esp_timer_get_time();
        // work out fuel_cons in L/s
        if (ALT_ECON_GAUGE_VAR) {
            fuel_cons = ALT_ECON_GAUGE_VAR * (out_speedo / 3600.0 / 100.0); // i.e. convert L/100km -> L/s
        }
        else {
            // check for refuel (or first boot), otherwise calculate fuel usage
            if (fuel > fuel_last) {
                fuel_cons = 0;

                fuel_last = fuel;
                fuel_time_last = fuel_time_now;
            }
            else {
                fuel_time_elapsed = fuel_time_now - fuel_time_last;
                // check a reasonable number before division (e.g. 1ms)
                if (fuel_time_elapsed > 1e3) {
                    fuel_cons = 1e6 * (fuel_last - fuel) / fuel_time_elapsed; // L/s
                }
            }
        }
        // just for testing
        if (out_speedo > 0) {
            Lp100km = update_ema(100 * fuel_cons * 3600 / out_speedo, Lp100km, out_econ_alpha);
        }
        
        out_econ = 255 * fuel_cons / max_fuel_flow; // scale to 0-255 based on approx injector size of E36 318i (21.5 lb/hr x4) (i.e. injector DC)
        float unsmoothed_out_econ = out_econ;
        out_econ = update_ema(out_econ, out_econ_last, out_econ_alpha);

        // sprintf(uart_buff, "time = %llu  L/100km = %.2f  out_econ = %.4f  unsmoothed_out_econ = %.4f", fuel_time_now, Lp100km, out_econ, unsmoothed_out_econ);echo_uart(uart_buff);

        out_econ_last = out_econ;
        fuel_last = fuel;
        fuel_time_last = fuel_time_now;

        ledc_set_freq(LED_SPEED_MODE, ECON_TIMER, out_tacho / (2 * ECON_DIVIDER)); // every 2nd engine revolution
        ESP_ERROR_CHECK(ledc_set_duty(LED_SPEED_MODE, ECON_CHAN, constrain(255 - unsmoothed_out_econ, econ_limit)));
        ESP_ERROR_CHECK(ledc_update_duty(LED_SPEED_MODE, ECON_CHAN));


        // update fuel
        float out_fuel = cal(fuel, fuel_fuel_map, fuel_limit);
        // sprintf(uart_buff, "fuel = %.4f  out_fuel = %.4f  out_fuel_V = %.4f", fuel, out_fuel, out_fuel * 5. / 255.);echo_uart(uart_buff); // this line causes some kerfuffle for some reason, fine for testing
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FUEL_CHAN, out_fuel));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FUEL_CHAN));

        // update temp
        float out_temp = cal_temp(temp, temp_Beta, temp_map);
        // sprintf(uart_buff, "temp_Beta = %.4f  temp = %i  out_temp = %.4f  out_temp_V = %.4f", temp_Beta, temp, out_temp, out_temp * 5. / 1024.);echo_uart(uart_buff); // this line causes some kerfuffle for some reason, fine for testing
        ESP_ERROR_CHECK(ledc_set_duty(LED_SPEED_MODE, TEMP_CHAN, constrain(out_temp, temp_limit)));
        ESP_ERROR_CHECK(ledc_update_duty(LED_SPEED_MODE, TEMP_CHAN));

        uart_flush(uart_num); // not sure this is needed, but just in case there is extra junk in the buffer(?)
    }
}


void app_main() {
    uart_init();
    echo_uart("Starting up.");

    temp_Beta = calc_temp_beta(temp_map);

    ledc_init();

    gauge_sweep();

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(watch_uart, "watch_uart_task", WATCH_UART_STACK_SIZE, NULL, 10, NULL);
}
