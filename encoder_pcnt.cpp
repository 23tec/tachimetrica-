// Encoder.cpp - Quadrature encoder on esp32
// Angelo Z. (2025)
#include <Arduino.h>
#include "driver/pcnt.h"

class Encoder {
private:
    int pinA;
    int pinB;
    pcnt_unit_t unit;
    pcnt_channel_t channel;
    int16_t count;
    int16_t lastCount;
    bool turned;

public:
    Encoder(int a, int b, pcnt_unit_t u, pcnt_channel_t ch) {
        pinA = a;
        pinB = b;
        unit = u;
        channel = ch;
        count = 0;
        lastCount = 0;
        turned = false;

        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);

        pcnt_config_t config = {
            .pulse_gpio_num = pinA,
            .ctrl_gpio_num = pinB,
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

        //pcnt_set_filter_value(unit, 10);
        //pcnt_filter_enable(unit);
        pcnt_filter_disable(unit);

        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
    }

    void update() {
        pcnt_get_counter_value(unit, &count);
        turned = (count != lastCount);
        lastCount = count;
    }

    bool hasTurned() const {
        return turned;
    }

    int16_t getCount() const {
        return count;
    }

    void reset() {
        pcnt_counter_clear(unit);
        count = 0;
        lastCount = 0;
        turned = false;
    }
};

Encoder encoderZ(21, 22, PCNT_UNIT_0, PCNT_CHANNEL_0);

void setup() {
  Serial.begin(115200);
}

void loop() {
  encoderZ.update();

  if (encoderZ.hasTurned()) {
    Serial.println(encoderZ.getCount());
  }
}