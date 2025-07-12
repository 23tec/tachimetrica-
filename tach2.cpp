// tach.cpp - Sistema a closed loop con feedback tachimetrica. 
// Angelo Z. (2025) 
#include <OneButton.h>
#include <PID_v1.h>
#include <SparkFun_External_EEPROM.h>
#include <Wire.h>


#define ARRAY_LEN(x)      (sizeof(x) / sizeof(x)[0])

#define READ_config       1 // Lettura eeprom
#define WRITE_config      0 // Scrittura eeprom

#define TACH_DYNAMO_PIN  A1 // Tachimetrica 
#define POT_PIN          A0 // Potenziometro
#define OPTO_PIN          2 // Fotoaccoppiatore su interrupt
#define TRIAC_PIN         5 // Relè a stato solido per pilotare i gate dei tiristori
#define BUZ_PIN           7 // Piezoelettrico

#define POT_RANGE       255 // Spectrum potenziometro 
#define SERIAL_INTERVAL  40 // Anti-flood per la ftdi, altrimenti disconnette
#define SERIAL_MAXBUF    30 // Buffer dati

#define INVERTED_SCALE(x) (POT_RANGE - (x)) // AC mode 

#define TACH_TIMEOUT    100 // Controllo segnale tachimetrica 
#define TACH_SIGNAL_MIN   1 // Valore fluttante su analog con cavo staccato 
#define SPIN_THRESHOLD   10 // Soglia sul potenziometro il motore inizia a spingere

// Tasti
OneButton buttonStart(9);
OneButton buttonStop(10);

// Buffer seriale
char receive[SERIAL_MAXBUF] = { 0 };
unsigned long serialDelay   = 0;

float   rampAccel               = 0.02;
float   rampDecel               = 0.05;

// Tachimetrica a 150 genera 2.90 volts
int     maxSpeed                = 150; // 0 - 255

volatile bool phaseZero         = false; 
volatile int  phaseAngle        = 0,
              phaseDelay        = 0;
              
bool          motorEnabled      = false,
              motorAccel        = false,
              alarm             = false;

double  kp = 1.40, 
        ki = 3.0,
        kd = 0.05,

        pidInput = 0.0, pidOutput = 0.0, pidSetPoint = 0.0;

// Pid logger
unsigned int listPid = 0;
unsigned long tachLastValid = 0;

// Lowpass filter
float alpha = 0.09;

// P_ON_M specifies that Proportional on Measurement be used.
// It make the output move more smoothly when the setpoint 
// is changed. 
// P_ON_E (Proportional on Error) is the default behavior
// DIRECT means the output will increase when error is positive.
// REVERSE is the opposite. 
PID pid(&pidInput, &pidOuput, &pidSetPoint, kp, ki, kd, P_ON_M, DIRECT);

ExternalEEPROM eeprom; 


#define M_ACCELERATION   0 
#define M_DECELARATION   1
#define M_INSPEED        2
#define M_TACH_ERR       3
#define M_RAMP           4
#define M_EEPROM_ERR     5
#define M_OPTO_ERR       6
#define M_IDLE           7

#define M_NONE        0xFF

const char M0[] PROGMEM = "Accelerazione in corso...";
const char M1[] PROGMEM = "Decelerazione in corso...";
const char M2[] PROGMEM = "In velocità rpm";
const char M3[] PROGMEM = "Tachimetrica segnale assente";
const char M4[] PROGMEM = "Rampa completata";
const char M5[] PROGMEM = "Memoria esterna non trovata";
const char M6[] PROGMEM = "Fotoaccoppiatore non risponde";
const char M7[] PROGMEM = "Motore fermo";
const char* const SERIAL_MSG[] PROGMEM = { M0, M1, M2, M3, M4, M5, M6, M7 };

byte sentMask = 0, mFlag = M_NONE;


inline bool isAlarmMessage() {
    return (mFlag == M_TACH_ERR   || 
            mFlag == M_EEPROM_ERR || 
            mFlag == M_OPTO_ERR);
}


void sendMessage() {
    if (mFlag == M_NONE || (sentMask & (1 << mFlag)) != 0 
        || mFlag >= ARRAY_LEN(SERIAL_MSG)) 
        return;
        
    if (alarm && !isAlarmMessage()) 
        return;
        
    Serial.println((PGM_P) pgm_read_ptr(&SERIAL_MSG[mFlag]));
    sentMask |= (1 << mFlag);
    
    mFlag = M_NONE;
}


void resetSent() {
    sentMask = 0; mFlag = M_NONE;
}


// Emetti suono
void beep(unsigned long duration) {
    tone( BUZ_PIN, 4000, duration );
}


// Riavvia microcontrollore 
void resetAVR() {
    Serial.end(); delay(1000);
    asm volatile ("jmp 0");
}


// Pulisci buffer seriale
void clearData() {
    memset(receive, 0, SERIAL_MAXBUF);
}


// Leggi dati seriale in arrivo
void serialReceive() {
    int i = 0;

    while (Serial.available() > 0 
    && i < SERIAL_MAXBUF - 1) 
        receive[i++] = Serial.read();

    if (i > 0) ATCommand(receive);
}


template <typename T> void eeFunc(bool read, int addr, T& var) {
  if (read) 
    eeprom.get(addr, var);
  else
    eeprom.put(addr, var);
}


// Alloca e legge dati su memoria esterna
void storage(unsigned int mode /* Write 0 - Read 1 */) {
    if (!eeprom.isConnected(0x50)) {
        mFlag = M_EEPROM_ERR;
        return;
    }
    
    /* Con polling abilitato (enablePollForWriteComplete) non occorre 
       un delay per ogni ciclo di scrittura. La eeprom imposta un bit WIP 
       a fine ciclo.
       eeprom.disablePollForWriteComplete();
       eeprom.setWriteTimeMs(7);
    */
    
    eeFunc(mode, 0, kp);          // float
    eeFunc(mode, 4, ki);          // float
    eeFunc(mode, 8, kd);          // float
    eeFunc(mode, 12, rampAccel);  // float
    eeFunc(mode, 16, rampDecel);  // float
    eeFunc(mode, 20, maxSpeed);   // int

    beep(200);
}


// Rimuove il carattere d'invio
char *removeLn(char *str) {
    size_t n = strcspn(str, "\n");
    if (n) *(str + n) = '\0';
    return str;
}


// Controlla validità numero
bool isValidNumber(const char *str) {
    if (str == NULL || *str == '\0') 
        return false;
        
    char *endptr;
    strtod(str, &endptr);
    return *endptr == '\0' && endptr != str;
}


// Lista commandi 
void AT_help() {
    const char *commandList[] = {
        "at",
        "at+kp=<float>",
        "at+ki=<float>",
        "at+kd=<float>",
        "at+pid=<kp,ki,kd>",
        "at+list=<int>",
        "at+accel=<float>",
        "at+decel=<float>",
        "at+reset",
        "at+write",
        "at+maxspeed=<0-255>"
    };

    for (size_t i = 0; i < ARRAY_LEN(commandList); i++) {
        Serial.println(commandList[i]);
        delay(100);
    }

    Serial.println("use at+name? for query setting");
}


// Parser per commandi seriali
bool AT_valid(char *cmd) {
    char tokens[10] = { 0 }, *argv = NULL;
    float values[3];
    bool accepted = false;
    int i = 0;


    // Test if serial connection response
    if (strcmp(cmd, "at") == 0) 
    {
        // My FTDI has DTR(Data terminal ready) and CTS(Clear to send) pin
        // I can connect one of these to any digitals pins as input signal
        // DTR is High serial is ready
        // CTS is High ready to receive data

        //if (digitalRead(ftdi_cts_pin) == HIGH) accepted = true;
        accepted = true;
    }

    // at+pid=kp,ki,kd
    if (strncmp(cmd, "at+pid=", 7) == 0) {
        strcpy(tokens, cmd + 7);
        
        argv = strtok(tokens, ",");

        while (argv && i < 3) {
            if (isValidNumber(argv))
                values[i++] = atof(argv);
            else
                break;
                
            argv = strtok(NULL, ",");
        }

        if (i == 3) 
        {
            kp = values[0];
            ki = values[1];
            kd = values[2];

            pid.SetTunings(kp, ki, kd)
            
            accepted = true;
        }
    }

    // Kp, Ki, Kd
    const char *v = cmd + 6;
    if (isValidNumber(v)) {
        if (strncmp(cmd, "at+kp=", 6) == 0) 
            kp = atof(v); accepted = true;
        if (strncmp(cmd, "at+ki=", 6) == 0)
            ki = atof(v); accepted = true;
        if (strncmp(cmd, "at+kd=", 6) == 0)
            kd = atof(v); accepted = true;
        if (accepted) pid.SetTunings(kp, ki, kd);
    }

    // Ramp
    v = cmd + 9;
    if (isValidNumber(v)) {
        if (strncmp(cmd, "at+accel=", 9) == 0) 
            rampAccel = atof(v); accepted = true;
        if (strncmp(cmd, "at+decel=", 9) == 0)
            rampDecel = atof(v); accepted = true;
    }

    // Max Speed 0-255
    v = cmd + 12;
    if (isValidNumber(v)) {
        int range = atoi(v);
        if (strncmp(cmd, "at+maxspeed=", 12) == 0) {
            if (range >= 0 && range <= POT_RANGE) {
                maxSpeed = range;
                accepted = true;
            }
        }
    }

    // Reboot avr
    if (strcmp(cmd, "at+reset") == 0) {
        Serial.println("OK");
        resetAVR();
    }

    // Write to eeprom
    if (strcmp(cmd, "at+write") == 0) {
        storage(WRITE_config);
        accepted = true;
    }

    // List pid
    if (strncmp(cmd, "at+list=", 8) == 0) {
        v = cmd + 8;
        if (isValidNumber(v)) {
            listPid = atoi(v);
            accepted = true;
        }
    }

    // Lowpass filter
    if (strncmp(cmd, "at+alpha=", 9) == 0) {
        v = cmd + 9;
        if (isValidNumber(v)) {
            if (alpha > 0.00) alpha = atof(v);
            accepted = true;
        }
    }

    // Help 
    if (strcmp(cmd, "at+help") == 0) AT_help();

    // Queries
    if (strcmp(cmd, "at+kp?") == 0) Serial.println(kp);
    if (strcmp(cmd, "at+ki?") == 0) Serial.println(ki);
    if (strcmp(cmd, "at+kd?") == 0) Serial.println(kd);
    if (strcmp(cmd, "at+accel?") == 0) Serial.println(rampAccel);
    if (strcmp(cmd, "at+decel?") == 0) Serial.println(rampDecel);
    if (strcmp(cmd, "at+maxspeed?") == 0) Serial.println(maxSpeed);


    return accepted;
}


// Commando inviato
void ATCommand(char *incomingData) {
    char *cmd = incomingData;

    if (AT_valid(cmd)) {
        Serial.println("OK");
    }
    
    clearData();
}



// Interrupt su fronte in salita
void ac_on() {
    // Spegni triac
    PORTD &= ~(1 << TRIAC_PIN);
    
    phaseAngle = INVERTED_SCALE(pidOutput);
    phaseDelay = 0;
    phaseZero  = true;
    
    interruptEvent = micros();
}


// Triac in conduzione nel angolo di fase
ISR(TIMER1_COMPA_vect) {
    if (phaseZero) {
        // Accendi triac
        if (phaseDelay >= phaseAngle) {
            PORTD |= (1 << TRIAC_PIN);
            phaseZero = false;
            phaseDelay = 0;
        } else {
            phaseDelay++;
        }
    }
}


// Attiva motore 
void startMotor() {
    if (motorEnabled) return;

    motorEnabled = motorAccel = true;
    beep(200);
    
    resetSent();
}


// Disattiva motore 
void stopMotor() {
    motorEnabled = motorAccel = false;
}


// Tasti in ascolto
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

    TCCR1A = 0;
    TCCR1B = 0; 
    TCNT1  = 0; 
    
    // 4us ((F_CPU / 64) * 0.040) - 1;
    // Con prescaler 8 abbiamo 79
    OCR1A = 9;
    // Clear timer on compare match e prescaler 64
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
    // Abilità interrupt su confronto 
    TIMSK1 |= (1 << OCIE1A);
    

    attachInterrupt( digitalPinToInterrupt( OPTO_PIN ), ac_on, RISING );

    Wire.begin();
    Wire.setClock( 400000 );
    eeprom.setMemoryType( 256 );
    eeprom.begin( 0x50 );

    // Load settings
    storage(READ_config);

    pid.SetSampleTime( 2 );
    pid.SetOutputLimits( 0, maxSpeed );
 

    beep(200);

    serialDelay = millis();
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Letture dalla tachimetrica
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



inline bool tachSignal() {
    return analogRead(TACH_DYNAMO_PIN) > TACH_SIGNAL_MIN;
}


inline bool optoIsolatorFault() {
    return micros() - interruptEvent >= 10000;
}


void loop() {
    int speedPot = map(analogRead(POT_PIN), 0, 1023, 0, maxSpeed);
    float tachPWM = readDynamo();

    
    if (optoIsolatorFault()) {
        mFlag = M_OPTO_ERR; 
        alarm = true;
    }
    

    if (tachSignal()) tachLastValid = millis();
    if (millis() - tachLastValid >= TACH_TIMEOUT &&
            speedPot >= SPIN_THRESHOLD) {
        mFlag = M_TACH_ERR;
        alarm = true;
    }
    
    
    if (!alarm) {
        pidInput = tachPWM;
        
        if (motorEnabled) {
            pid.SetMode(AUTOMATIC);
            
            // In accelerazione
            if (motorAccel) {
                pidSetPoint = min(speedPot, pidSetPoint + rampAccel);
            
                // Fine rampa
                if (pidInput >= speedPot) {
                    motorAccel = false;
                    mFlag = M_RAMP;
                } else {
                    mFlag = M_ACCELERATION;
                }
            } else {
                pidSetPoint = speedPot;
                mFlag = M_INSPEED;
            }

            // Segnalazione sotto sforzo
            if (!motorAccel && pidInput - pidSetPoint <= -8.0) 
                beep(100);
        } else {
            // In decelerazione
            pidSetPoint = max(0, pidSetPoint - rampDecel);
    
            // Fine rampa
            if (pidSetPoint <= 0.0) {
                mFlag = M_RAMP;
        
                if (!tachSignal() && pidOutput <= 0.5) {
                    pid.SetMode(MANUAL);
                    mFlag = M_IDLE;
                }
            } else {
                mFlag = M_DECELARATION;
            }
        }
    }


    if (alarm) {
        pid.SetMode(MANUAL);
        
        motorEnabled = false;
        motorAccel   = false;
        
        pidInput     = 0;
        pidSetPoint  = 0;
        pidOutput    = 0;
    } else {
        pid.Compute();
    }
    

    if (Serial) {
        if (millis() - serialDelay >= SERIAL_INTERVAL) {
            serialReceive();

            if (listPid > 0) {
                Serial.print("Pot: "); Serial.print(speedPot);
                Serial.print("  Tach: "); Serial.println(pidInput);
                listPid--;
            } else {
                sendMessage();
            }
            
            serialDelay = millis();
        }
    }

    buttons_listening();
}


// eof