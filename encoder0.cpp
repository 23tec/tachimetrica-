// Encoder.cpp - Lettura encoder a quadratura con esp32-devkit v1
// Angelo Z. (2025)

// I Encoders vanno usati con una accelerazione e non girati bruscamente 
#include <Arduino.h>
#include "driver/pcnt.h"

#define BAUDRATE 115200

#define TOUCHDRO_TIMEOUT    500
#define GLITCH_MS_DEFAULT   500
#define POWER_LED_PIN        12
#define HALLSENSOR_PIN        4
#define TACH_REFRESH_RATE   500 
#define POLES                 1

class Encoder {
private:
    int pin_a;
    int pin_b;
    pcnt_unit_t unit;
    pcnt_channel_t channel;
    int16_t count;
    int16_t lastCount;
    bool turn;

public:
    Encoder(int a, int b, pcnt_unit_t ui, pcnt_channel_t ch) {
        pin_a = a;
        pin_b = b;
        unit = ui;
        channel = ch;
        count = 0;
        lastCount = 0;
        turn = false;

        pinMode(pin_a, INPUT);
        pinMode(pin_b, INPUT);

        pcnt_config_t config = {
            .pulse_gpio_num = pin_a,
            .ctrl_gpio_num = pin_b,
            .lctrl_mode = PCNT_MODE_REVERSE,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DEC,
            .counter_h_lim = 32767,
            .counter_l_lim = -32768,
            .unit = unit,
            .channel = channel
        };

        pcnt_unit_config(&config);

        //pcnt_set_filter_value(unit, GLITCH_MS_DEFAULT);
        //pcnt_filter_enable(unit);
        pcnt_filter_disable(unit);

        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
    }

    void update() {
        pcnt_get_counter_value(unit, &count);
        turn = (count != lastCount);
        lastCount = count;
    }

    bool isMoving() const {
        return turn;
    }

    int16_t getCount() const {
        return count;
    }

    void reset() {
        pcnt_counter_clear(unit);
        count = 0;
        lastCount = 0;
        turn = false;
    }
};


// Asse Z
Encoder encoder0(22, 23, PCNT_UNIT_0, PCNT_CHANNEL_0);
// Asse X
Encoder encoder1(18, 19, PCNT_UNIT_1, PCNT_CHANNEL_1);

// Touchdro timeout
unsigned long lastSendTime = 0;
// Tach signal
volatile uint32_t sensorPulses = 0;
unsigned long lastRpmTime = 0;
int16_t lastRpm = 0;


void IRAM_ATTR tachISR() { sensorPulses++; }

void setup() {
  Serial.begin(BAUDRATE);

  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(HALLSENSOR_PIN, INPUT_PULLUP);
    
  attachInterrupt(HALLSENSOR_PIN, tachISR, FALLIMG);

  digitalWrite(POWER_LED_PIN, HIGH);
  lastRpmTime = millis();
}


// Touchdro string format Z0; X0; T0;
void serialSend(int16_t dataBlock, char prefix) {
    Serial.print(prefix); 
    Serial.print(dataBlock);
    Serial.println(";");
}


int16_t getRpm() {
    unsigned long now = millis();
    unsigned long deltaTime = now - lastRpmTime;
    int16_t rpm = lastRpm;

    if (deltaTime >= TACH_REFRESH_RATE && deltaTime > 0) {
        rpm = (sensorPulses * 60000UL) / (deltaTime * POLES);
        sensorPulses = 0;
        lastRpmTime = now;

        // Media mobile semplice
        rpm = (rpm + lastRpm) / 2;
        //rpm = (rpm * 0.7) + (lastRpm * 0.3);
        lastRpm = rpm;
    }

    return rpm;
}


void loop() {
  int16_t rpm = getRpm();
  
  // Touchdro disconnette dopo un periodo se non arrivano dati
  if (millis() - lastSendTime >= TOUCHDRO_TIMEOUT
      || encoder0.isMoving()
      || encoder1.isMoving()
      || rpm != 0) {
      
    serialSend(encoder0.getCount(), 'Z');
    serialSend(encoder1.getCount(), 'X');
    serialSend(rpm, 'T');
    
    lastSendTime = millis();
  }
  
  encoder0.update();
  encoder1.update();
}
