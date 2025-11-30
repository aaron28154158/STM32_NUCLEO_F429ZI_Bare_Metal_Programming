/*
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *
 */

#include <SPI.h>
#include <stdint.h>

#define SPI_SCK     13
#define SPI_MISO    12
#define SPI_MOSI    11
#define SPI_SS      10

char dataBuff[500];
uint16_t dataLen = 0;
uint32_t i = 0;

void SPI_SlaveInit(void)
{
    pinMode(SCK, INPUT);
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SS, INPUT);

    SPCR = (1 << SPE);
}

uint8_t SPI_SlaveReceive(void)
{
    while(!(SPSR & (1 << SPIF)));

    return SPDR;
}

void SPI_SlaveTransmit(char data)
{
    SPDR = data;

    while(!(SPSR & (1 << SPIF)));
}

void setup()
{
    Serial.begin(9600);

    SPI_SlaveInit();

    Serial.println("Slave Initialized");
}


void loop()
{
    Serial.println("Slave waiting for SS to go low");

    while(digitalRead(SS));

    dataLen = SPI_SlaveReceive();

    for(i = 0; i < dataLen; i++)
    {
        dataBuff[i] = SPI_SlaveReceive();
    }

    dataBuff[i] = '\0';

    Serial.println("Rcvd:");
    Serial.println(dataBuff);
    Serial.print("Length: ");
    Serial.println(dataLen);
}