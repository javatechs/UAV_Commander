#include <SoftwareSerial.h>
#include <dht.h>
#include <SPI.h>

//For SPI Slave
enum SPI_command_t{
  NO,
  STATUS,
  TEMPERATURE,
  MOISTURE,
  SOIL
};

enum SPI_status_t{
  SPI_IDLE,
  SPI_SENDING,
  SPI_RECEIVING
};

float Temp_value=0;
float Mois_value=0;
float Soil_value=0;
char rx_buffer[20];
volatile int8_t rx_index=0;
volatile char tx_buffer[20];
volatile int8_t tx_index=0;
volatile int8_t process_it=0;
volatile SPI_command_t SPI_command=NO;
volatile SPI_status_t SPI_status=SPI_RECEIVING;
volatile int8_t tx_length=0;


//For Xbee
SoftwareSerial mySerial(8, 9); // RX, TX

//For Soil sensor
int Soil_pin=A0;
int Soil_read=0;
int Soil_power=7;

//for Moisture sensor
#define DHT_PIN 5
dht DHT;
int DHT_chk=0;


void setup() {
  // put your setup code here, to run once:
  pinMode(Soil_power,OUTPUT);
  digitalWrite(7,LOW);
  pinMode(10,INPUT);
  Serial.begin(9600);
  mySerial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Soil reading
  digitalWrite(Soil_power,HIGH);
  delay(100);
  Soil_read=analogRead(Soil_pin);
  Serial.print("Analog read is: ");
  Serial.println(Soil_read*5.0/1024);
  mySerial.print("Analog read is: ");
  mySerial.println(Soil_read*5.0/1024);
  digitalWrite(Soil_power,LOW);

  //DHT reading
  DHT_chk=DHT.read(DHT_PIN);
  Serial.print("Temp is: ");
  Serial.print(DHT.temperature, 1);
  Serial.println("");

  delay(5000);   //update every 5 seconds
}
