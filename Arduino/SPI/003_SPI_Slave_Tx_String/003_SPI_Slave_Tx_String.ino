/*
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *
 */
 
#include <SPI.h>

String inputString = "";
bool stringComplete = false;
uint8_t t_buffer[200];
uint32_t cnt = 0;

void SPI_SlaveInit(void) 
{ 
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  SPCR = (1 << SPE);
}

void SPI_SlaveTransmit(uint8_t data)
{
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

#define DATA_READY_PIN 8
#define BUFFER_SIZE 200

void setup() {
  Serial.begin(9600);
  pinMode(DATA_READY_PIN, OUTPUT);
  digitalWrite(DATA_READY_PIN, LOW);
  SPI_SlaveInit();
  Serial.println("Ready");
}

void loop() {
  if(stringComplete) {
    digitalWrite(DATA_READY_PIN, HIGH);
    delayMicroseconds(100);
    
    unsigned long timeout = millis() + 1000;
    while(digitalRead(SS)) {
      if(millis() > timeout) {
        cnt = 0;
        stringComplete = false;
        digitalWrite(DATA_READY_PIN, LOW);
        Serial.println("Timeout: SS not LOW");
        return;
      }
    }
    
    SPI_SlaveTransmit(0xf1);
    
    for(uint32_t i = 0; i <= cnt; i++) { 
      SPI_SlaveTransmit(t_buffer[i]);
    }
    
    cnt = 0;
    stringComplete = false;
    digitalWrite(DATA_READY_PIN, LOW);
    Serial.println("Sent");
  }
}

void serialEvent() {
  while (Serial.available() && cnt < (BUFFER_SIZE - 1)) {
    char inChar = (char)Serial.read();
    t_buffer[cnt++] = inChar;
    
    if (inChar == '\r') {
      t_buffer[cnt] = '\0';  
      stringComplete = true;
      break;
    }
  }
}