#include "e36cluster.h"

// define pins
#define PIN_TACHO 32
#define PIN_SPEEDO 4
#define PIN_FUEL 5
#define PIN_TEMP 6
#define PIN_ECON 6

// gauge calibrations
// inputs to map() e.g.{output1, val1, output2, val2}
struct gauge_cal_points {
    const int inLo, outLo, inHi, outHi;
    const int outMin, outMax;
};
gauge_cal_points tacho_range = {31, 400, 307, 10000};
gauge_cal_points speedo_range = {30, 25, 392, 300};
gauge_cal_points fuel_range = {1, 0, 173, 60};
gauge_cal_points temp_range = {100, 0, 6, 200};
gauge_cal_points econ_range = {255, 0, 4, 30};


// gauge limits (used with constrain() for raw outputs)
struct gauge_limit {
    const int min, max;
};
gauge_limit tacho_limit = {31, 307}; // tone frequency
gauge_limit speedo_limit = {30, 392}; // tone frequency
gauge_limit fuel_limit = {1, 173}; // PWM duty cycle
gauge_limit temp_limit = {100, 6}; // PWM duty cycle
gauge_limit econ_limit = {255, 4}; // PWM duty cycle


int cal(int input, const gauge_cal_points& c, const gauge_limit& l) {
    int output = map(input, c.outLo, c.outHi, c.inLo, c.inHi);
    return constrain(output, l.min, l.max);
}

//PWM and tone constants
#define PWM_RES 8
#define TONE_DC 50

#define BYTE_SIZE sizeof(E36ClusterData)

E36ClusterData sd;

int rpms = 0;
int velocity = 0;
int wheelspeed = 0;
int gas = 0;
int fuel = 0;
int tyretemp = 0;

int out_tacho = tacho_limit.min;
int out_speedo = speedo_limit.min;
int out_fuel = fuel_limit.min;
int out_temp = temp_limit.min;
int out_econ = econ_limit.min;

void setup() {
    
    Serial.begin(115200);

    ledcAttach(PIN_TACHO, TONE_DC, PWM_RES);

    delay(100);
    
    ledcWriteTone(PIN_TACHO, out_tacho);

    //tone_tach.begin(PIN_TACH);
    //tone_speedo.begin(PIN_SPEEDO);
    //pinMode(PIN_FUEL, OUTPUT);
    //pinMode(PIN_TEMP, OUTPUT);
    //pinMode(PIN_ECON, OUTPUT);

    //tone_tach.play(MIN_FREQ_TACH);
    //tone_speedo.play(MIN_FREQ_SPEEDO);
    //analogWrite(PIN_FUEL, 160);
    //analogWrite(PIN_TEMP, 100);
    //analogWrite(PIN_ECON, 255);
    
    //TODO: gauge sweep
}

void loop() {
    char buff[BYTE_SIZE];

    if (Serial.available() >= BYTE_SIZE)
    {
        // read in serial data
        Serial.readBytes(buff, BYTE_SIZE);
        memcpy(&sd, &buff, BYTE_SIZE);

        rpms = sd.rpms;
        velocity = sd.velocity;
        wheelspeed = sd.wheelspeed; // can use when tyrediameter is defined (currently set to -1)
        gas = sd.gas;
        fuel = sd.fuel;
        tyretemp = sd.tyretemp;

        // update tach
        out_tacho = cal(rpms, tacho_range, tacho_limit);
        ledcWriteTone(PIN_TACHO, out_tacho);
        //out_freq_rpms = rpms * rpm2freq + MIN_FREQ_TACH;
        //out_freq_rpms = min(out_freq_rpms, MAX_FREQ_TACH);
        //out_freq_rpms = max(out_freq_rpms, MIN_FREQ_TACH);
        //out_freq_rpms = map(rpms, 1000, 7000, 50, 350);
        //out_freq_rpms = constrain(out_freq_rpms, MIN_FREQ_TACH, MAX_FREQ_TACH);
        //tone_tach.play(out_freq_rpms);

        // update speedo
        //out_freq_velocity = wheelspeed * velocity2freq + MIN_FREQ_SPEEDO;
        //out_freq_velocity = min(out_freq_velocity, MAX_FREQ_SPEEDO);
        //out_freq_velocity = max(out_freq_velocity, MIN_FREQ_SPEEDO);
        //tone_speedo.play(out_freq_velocity);


        // update fuel
        //int fuel_test = gas * 170 / 255;
        //fuel_test = map(gas, 0, 255, 0, 160);
        //fuel = constrain(fuel, 0, 60);
        //out_fuel = map(fuel, 0, 60, 1, 173);
        //analogWrite(PIN_FUEL, out_fuel);

        // update temp
        //int temp_test = ((255 - gas) * 150 / 255) + 6;
        //int temp_test = map(gas, 0, 255, 150, 6);
        //tyretemp = constrain(tyretemp, 150, 200); //normally min 0
        //out_tyretemp = map(tyretemp, 0, 200, 150, 6);
        //analogWrite(PIN_TEMP, out_tyretemp);

        // update econ
        //econ_test = 255-gas;
        //econ_test = map(gas, 0, 255, 200, 4);
        //analogWrite(PIN_ECON, econ_test);

        //delay(10);
    }
}
