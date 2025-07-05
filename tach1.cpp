// tach.cpp - Sistema a closed loop con feedback tachimetrica. 
// Angelo Z. (2025) 
#include <OneButton.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <SparkFun_External_EEPROM.h>
#include <Wire.h>

#define ARRAYLEN(x)        (sizeof(x) / sizeof(x)[0])

#define READ_config     1
#define WRITE_config    0

#define DYNAMO_PIN       A1
#define POT_PIN          A0
#define OPTO_PIN          2
#define TRIAC_PIN         5
#define POT_RANGE       255
#define SERIAL_INTERVAL  40
#define SERIAL_MAXBUF    30
#define INVERTED_SCALE(x) (POT_RANGE - (x)) // AC mode
#define TACH_TIMEOUT    100
#define TACH_MIN          1 // Analog
#define SPIN_THRESHOLD   10 

OneButton buttonStart(9);
OneButton buttonStop(10);

// Buffer seriale
char receive[SERIAL_MAXBUF] = { 0 };
unsigned long t1 = 0;

float   rampAccel              = 0.02;
float   rampDecel              = 0.05;

// Tachimetrica a 150 genera 2.90 volts
int     maxSpeed               = 150; // 0 - 255

volatile bool zero             = false;
volatile int  speedlevel       = 0,
              firingDelay      = 0;

bool          motorEnabled     = false,
              motorAccel       = false,
              alarm            = false;

double  kp = 1.40, 
        ki = 3.0,
        kd = 0.05,

        input = 0.0, output = 0.0, setpoint = 0.0;

unsigned int pidstat = 0;
unsigned long tachLastValid = 0;
float alpha = 0.09; // Lowpass filter

PID pid(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);

ExternalEEPROM eeprom;


void beep() {
    tone( 7, 4000, 200 );
}


void resetAVR() {
    if (Serial) Serial.end(); delay(100);
    asm volatile ("jmp 0");
}


void clearData() {
    memset(receive, 0, SERIAL_MAXBUF);
}


void serialReceive() {
    int i = 0;

    while (Serial.available() > 0 
    && i < SERIAL_MAXBUF - 1) 
        receive[i++] = Serial.read();

    if (i > 0) ATCommand(receive);
}


template <typename T> void ee(bool read, int addr, T& var) {
  if (read) 
    eeprom.get(addr, var);
  else
    eeprom.put(addr, var);
}


void storage(int rw) {
    if (!eeprom.isConnected(0x50)) {
        if (Serial) Serial.println("Memoria non rilevata");
        return;
    }
    
    /* Con polling abilitato (enablePollForWriteComplete) non serve
       un delay per ogni scrittura. La eeprom imposta un bit WIP 
       dopo fine ciclo.
       eeprom.disablePollForWriteComplete();
       eeprom.setWriteTimeMs(7);
    */
    
    ee(rw, 0, kp); // float
    ee(rw, 4, ki); // float
    ee(rw, 8, kd); // float
    ee(rw, 12, rampAccel); // float
    ee(rw, 16, rampDecel); // float
    ee(rw, 20, maxSpeed); // int

    beep();
}


char *removeLn(char *str) {
    size_t n = strcspn(str, "\n");
    if (n) *(str + n) = '\0';
    return str;
}


bool isValidNumber(const char *str) {
    bool hasDigits = false;
    bool hasDecimalPoint = false;

    while (*str) {
        if (isdigit(*str)) 
            hasDigits = true;
        else if (*str == '.') {
            if (hasDecimalPoint) return false;
            hasDecimalPoint = true;
        } else {
            return false;
        }

        str++;
    }

    return hasDigits;
}


void AT_help() {
    const char *commandList[] = {
        "at",
        "at+kp=<float>",
        "at+ki=<float>",
        "at+kd=<float>",
        "at+pid=<kp,ki,kd>",
        "at+showpid=<int>",
        "at+accel=<float>",
        "at+decel=<float>",
        "at+reset",
        "at+write",
        "at+maxspeed=<0-255>"
    };

    for (size_t i = 0; i < ARRAYLEN(commandList); i++) {
        Serial.println(commandList[i]);
        delay(100);
    }

    Serial.println("use at+name? for query setting");
}


bool AT_valid(char *prompt) {
    char tokenBuffer[10] = { 0 };
    float values[3];
    bool accepted = false;
    int i = 0;

    // Test if serial connection response
    if (strcmp(prompt, "at") == 0) 
    {
        // My FTDI has DTR(Data terminal ready) and CTS(Clear to send) pin
        // I can connect one of these to any digitals pins as input signal
        // DTR is High serial is ready
        // CTS is High ready to receive data

        //if (digitalRead(ftdi_cts_pin) == HIGH) accepted = true;
        accepted = true;
    }

    // at+pid=0,0,0
    if (strncmp(prompt, "at+pid=", 7) == 0) {
        strcpy(tokenBuffer, prompt + 7);
        char *arg = strtok(tokenBuffer, ",");

        while (arg && i < 3) {
            if (isValidNumber(arg))
                values[i++] = atof(arg);
            arg = strtok(NULL, ",");
        }

        if (i == 3) 
        {
            kp = values[0];
            ki = values[1];
            kd = values[2];

            accepted = true;
        }
    }

    // Kp, Ki, Kd
    const char *v = prompt + 6;
    if (isValidNumber(v)) {
        if (strncmp(prompt, "at+kp=", 6) == 0) 
            kp = atof(v); accepted = true;
        if (strncmp(prompt, "at+ki=", 6) == 0)
            ki = atof(v); accepted = true;
        if (strncmp(prompt, "at+kd=", 6) == 0)
            kd = atof(v); accepted = true;
    }

    // Ramp
    v = prompt + 9;
    if (isValidNumber(v)) {
        if (strncmp(prompt, "at+accel=", 9) == 0) 
            rampAccel = atof(v); accepted = true;
        if (strncmp(prompt, "at+decel=", 9) == 0)
            rampDecel = atof(v); accepted = true;
    }

    // Speed 0-255
    v = prompt + 12;
    if (isValidNumber(v)) {
        int pwm = atoi(v);
        if (strncmp(prompt, "at+maxspeed=", 12) == 0) {
            if (pwm >= 0 && pwm <= POT_RANGE) {
                maxSpeed = pwm;
                accepted = true;
            }
        }
    }

    // Reboot avr
    if (strcmp(prompt, "at+reset") == 0) {
        Serial.println("OK");
        resetAVR();
    }

    // Write to eeprom
    if (strcmp(prompt, "at+write") == 0) {
        storage(WRITE_config);
        accepted = true;
    }

    // List pid
    if (strncmp(prompt, "at+showpid=", 11) == 0) {
        v = prompt + 11;
        if (isValidNumber(v)) {
            pidstat = atoi(v);
            accepted = true;
        }
    }

    // Lowpass filter
    if (strncmp(prompt, "at+alpha=", 9) == 0) {
        v = prompt + 9;
        if (isValidNumber(v)) {
            alpha = atof(v);
            accepted = true;
        }
    }

    // Help 
    if (strcmp(prompt, "at+help") == 0) AT_help();

    // Queries
    if (strcmp(prompt, "at+kp?") == 0) Serial.println(kp);
    if (strcmp(prompt, "at+ki?") == 0) Serial.println(ki);
    if (strcmp(prompt, "at+kd?") == 0) Serial.println(kd);
    if (strcmp(prompt, "at+accel?") == 0) Serial.println(rampAccel);
    if (strcmp(prompt, "at+decel?") == 0) Serial.println(rampDecel);
    if (strcmp(prompt, "at+maxspeed?") == 0) Serial.println(maxSpeed);

    return accepted;
}


void ATCommand(char *prompt) {
    char *cmd = prompt;

    if (AT_valid(cmd)) Serial.println("OK");

    clearData();
}


void ac_on() {
    zero = true;
    firingDelay = 0;
    // Spegni triac
    PORTD &= ~(1 << TRIAC_PIN);
    speedlevel = INVERTED_SCALE(output);
}


void setSpeed() {
    if (zero == false) return;
    if (firingDelay >= speedlevel) {
        // Accendi triac
        PORTD |= (1 << TRIAC_PIN);
        zero = false;
        firingDelay = 0;
    } else firingDelay++;
}


void startMotor() {
    if (motorEnabled) return;

    motorEnabled = motorAccel = true;
    beep();
}


void stopMotor() {
    motorEnabled = motorAccel = false;
}


void buttons_listening() {
    buttonStart.tick();
    buttonStop.tick();
}


void setup() {
    Serial.begin( 115200 );
 
    pinMode( OPTO_PIN, INPUT );
    pinMode( TRIAC_PIN, OUTPUT );
    digitalWrite( TRIAC_PIN, LOW );

    buttonStart.attachClick( startMotor );
    buttonStop.attachClick( stopMotor );

    Timer1.initialize( 40 );
    Timer1.attachInterrupt( setSpeed );

    attachInterrupt( digitalPinToInterrupt( OPTO_pin ), ac_on, RISING );

    
    Wire.begin();
    Wire.setClock( 400000 );
    eeprom.setMemoryType( 256 );
    eeprom.begin( 0x50 );

    // Load settings
    storage(READ_config);

    pid.SetSampleTime( 2 );
    pid.SetOutputLimits( 0, maxSpeed );
 

    beep();

    t1 = millis();
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//Letture dalla tachimetrica
float readDynamo() {
    const int numReadings = 50;
    static float readings[numReadings] = { 0 };
    static unsigned int readIndex = 0;
    static float total = 0;

    static float filtered = 0.0;
    

    total = total - readings[readIndex];
    readings[readIndex] = mapf(analogRead(DYNAMO_PIN), 0.0, 1023.0, 0.0, POT_RANGE);
    total = total + readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    
    float average = total / numReadings;

    filtered += alpha * (average - filtered);

    return filtered;
}


// Con cavo staccato leggere il valore restituito sulla analogica su DYNAMO_PIN
bool tachIsDisconnected() {
    const int RAW_MIN_ANALOG = TACH_MIN;
    const int raw = analogRead(DYNAMO_PIN);
    
    return raw >= RAW_MIN_ANALOG;
}


void loop() {
    int speedPot = map(analogRead(POT_PIN), 0, 1023, 0, maxSpeed);
    float tachPWM = readDynamo();

    input = tachPWM;
    
    
    // Start button
    if (motorEnabled && !alarm) {
        pid.SetMode(AUTOMATIC);

        if (!tachIsDisconnected()) tachLastValid = millis();
            
        // Evitare che pid mandi il motore a massima potenza se
        // non arriva segnale dalla tachimetrica 
        if (millis() - tachLastValid >= TACH_TIMEOUT 
                && speedPot >= SPIN_THRESHOLD) {
            if (Serial) Serial.println("Errore: Tachimetrica segnale assente");
            alarm = true;
        } else {
            // In accelerazione
            if (motorAccel) {
                setpoint = min(speedPot, setpoint + rampAccel);

                if (input >= speedPot) 
                    motorAccel = false;
            } else {
                // In velocità stabile
                setpoint = speedPot;
            }

            // Segnalazione sotto sforzo
            if (!motorAccel && input - setpoint <= -8.0) 
                beep();
        }
    // Stop button
    } else {
        // In decelerazione
        setpoint = max(0, setpoint - rampDecel);

        if (setpoint <= 0.0) {
            pid.SetMode(MANUAL);
            
            output = input = 0.0;
        }
    }

    if (!alarm) {
        pid.SetTunings(kp, ki, kd);
        pid.Compute();
    } else {
        pid.SetMode(MANUAL);
        
        motorEnabled = false;
        motorAccel   = false;
        setpoint     = 0;
        input        = 0;
        output       = 0;
    }


    if (Serial) {
        if (millis() - t1 >= SERIAL_INTERVAL) {
            serialReceive();

            if (pidstat > 0) {
                Serial.print("Pot: "); Serial.print(speedPot);
                Serial.print("  Tach: "); Serial.println(input);
                pidstat--;
            }

    
            t1 = millis();
        }
    }

    buttons_listening();
}


// eof