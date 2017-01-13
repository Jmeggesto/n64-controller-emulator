#include <Arduino.h>
#include <SPI.h>

const int pin = 12;

// SEQLEN:
// Response buffer will contain
// 16 bytes of potentially-full data plus 
// one additional byte, of which
// only the 4 most significant bits are
// of importance for their use in the stop bit

#define SEQLEN 17 
#define BUFFLEN 4

/*

Reference for buttons

 A 0
 B 1
 Z 2
 S 3
 dU 4
 dD 5
 dL 6
 dR 7
 rst 8
 L 10
 R 11
 cU 12
 cD 13
 cL 14
 cR 15
 */
 #define X 16
 #define Y 24


#define MSNH 0x71 // most significant nibble high
#define LSNH 0x17 // least significant nibble high

#define BYTE_FULL 0x77
#define BYTE_EMPTY 0x11
#define STOP_BIT 0x30

volatile bool sentLast;

volatile int8_t bttn1;
volatile int8_t bttn2;
volatile int8_t xAxis;
volatile int8_t yAxis;

IntervalTimer setCommand;

SPISettings settings(1000000, MSBFIRST, SPI_MODE1); 

int8_t seq[SEQLEN];
int8_t emptyAction[SEQLEN];

void setup() {

    init();
    Serial.begin(115200);
    Serial.print("Starting Program");
    Serial.println();
    delay(500);
    Serial.print("Ready");
    Serial.println();
    attachInterrupt(digitalPinToInterrupt(pin), writeSeq, FALLING);
    setCommand.priority(0);
    setCommand.begin(nextCommand, 400);
    SPI.begin();
}

void loop() {

    int count = 0;
    int8_t buff[BUFFLEN];
    while (count<BUFFLEN) {
        if (Serial.available() && sentLast) {
            buff[count++] = Serial.read();
        }
    }
    bttn1 = buff[0];
    bttn2 = buff[1];
    xAxis = buff[2];
    yAxis = buff[3];
    sentLast = false;
}

void init() {

    for (int i = 0; i < 16; i ++) {
        emptyAction[i] = BYTE_EMPTY;
    }

    // Stop Bit
    emptyAction[16] = STOP_BIT;
    resetSeq();

    sentLast = true;
    bttn1 = 0;
    bttn2 = 0;
    xAxis = 0;
    yAxis = 0;

    pinMode(pin, INPUT);
    // Set pin interrupt
}

void writeSeq() {

    noInterrupts();
    setCommand.end();
    delayMicroseconds(32);

    pinMode(pin, OUTPUT);

    SPI.beginTransaction(settings);
    for(int8_t i : seq) {
        SPI.transfer(i);
    }
    SPI.endTransaction();

    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), writeSeq, FALLING);
    setCommand.begin(nextCommand, 4000);
    interrupts();
}

void nextCommand() {

    setCommand.end();
    resetSeq();

    for (int i=0 ; i <8 ; i++){
        if ((0xFF>>i) & bttn1) {
            pressButton(i);
        }
        if ((0xFF>>i) & bttn2) {
            pressButton(i + 8);
        }
        if ((0xFF>>i) & xAxis) {
            pressButton(i + 16);
        }
        if ((0xFF>>i) & yAxis) {
            pressButton(i + 24);
        }        
    }

    bttn1 = 0x00;
    bttn2 = 0x00;
    xAxis = 0x00;
    yAxis = 0x00;

    //Serial.print('p');
    sentLast = true;

}

void pressButton(int button) {
    int idx = button / 2;
    if(button % 2 == 0) {
        seq[idx] |= MSNH;
    } else {
        seq[idx] |= LSNH;
    }
}

void resetSeq() {
    for (int i=0 ; i<SEQLEN ; i++) {
        seq[i] = emptyAction[i];
    }
}
