#include <SoftwareSerial.h>
#include <dht.h>

//#define DEBUG

//For Xbee
SoftwareSerial mySerial(8, 9); // RX, TX
double Temp_value=0;
double Humi_value=0;
double Soil_value=0;
enum Xbee_command_t{
  TEMP,
  HUMI,
  SOIL
};
boolean process_it=false;
char rx_buffer[20];
char tx_buffer[10];
uint8_t rx_index=0;
int8_t tx_length=0;

//For Soil sensor
int Soil_pin=A0;
int Soil_read=0;
int Soil_power=7;

//for Moisture sensor
#define DHT_PIN 5
dht DHT;
int DHT_chk=0;

void xbee_print_package(char*tx){
  mySerial.print(tx[0]);
  mySerial.print(tx[1]);
  mySerial.print(tx[2]);
  mySerial.print(tx[3]);
  mySerial.print(tx[4]);
  mySerial.print(tx[5]);
}

void tx_data_serialize(int16_t data, char* tx, int8_t* len){
  int8_t payload_len = 2;
  int8_t temp_len = payload_len+3;
  int8_t i=0;
  tx[0]=0x7F;   //start char
  tx[1]=2;
  tx[2]=(data>>8)&0xff;
  tx[3]=(data&0xff);
  *len = temp_len;
  //checksum as 0xff first
  tx[4]=0xff;
  tx[5]=0x00;
  
}

void process_xbee_request(){
  //Serial.print("Process: ");
  //Serial.println(rx_buffer);

    //Humidity
  if(strcmp(rx_buffer, "HUMIDITY")==0){
    Serial.println("Process HUMIDITY");
    int16_t humi16=(int16_t)(Humi_value*10);
    tx_data_serialize(humi16, tx_buffer, &tx_length);
    xbee_print_package(tx_buffer);
  }

  
  //Temperature
  if(strcmp(rx_buffer, "TEMPERATURE")==0){
    Serial.println("Process TEMPERATURE");
    int16_t temp16=(int16_t)(Temp_value*10);
    tx_data_serialize(temp16, tx_buffer, &tx_length);
    xbee_print_package(tx_buffer);
    Serial.print(temp16);
  }



  //Soil
  if(strcmp(rx_buffer, "SOIL")==0){
    Serial.println("Process SOIL");
    int16_t soil16=(int16_t)(Soil_value*10);
    tx_data_serialize(soil16, tx_buffer, &tx_length);
    xbee_print_package((char*)tx_buffer);
  }

  //debug
  #ifdef DEBUG
  if(rx_buffer[0]==9){
    Serial.println("Process Debug");
    int16_t soil16=(int16_t)(Soil_value*10);
    tx_data_serialize(soil16, tx_buffer, &tx_length);
    mySerial.print((char*)tx_buffer);
  }
  #endif
  
  rx_index=0;
  memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
}

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
  #ifdef DEBUG
  Serial.print("Analog read is: ");
  Serial.println(Soil_read*5.0/1024);
  #endif
  Soil_value=Soil_read*5.0/1024;
  digitalWrite(Soil_power,LOW);

  //DHT:temp reading
  DHT_chk=DHT.read(DHT_PIN);
  Temp_value=DHT.temperature;
  #ifdef DEBUG
  Serial.print("Temp is: ");
  Serial.print(DHT.temperature, 1);
  Serial.println("");
  #endif

  //DHT:humi reading
  Humi_value=DHT.humidity;
  #ifdef DEBUG
  Serial.print("Temp is: ");
  Serial.print(DHT.humidity, 1);
  Serial.println("");
  #endif

  //Read XBEE
  while (mySerial.available()) {
    // get the new byte:
    char inChar = (char)mySerial.read();
    rx_buffer[rx_index]=inChar;
    rx_index++;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\0') {
      process_it = true;
      rx_index=0;
    }
    if(rx_index>=20){
      rx_index=0;
    }
  }
  
  //XBEE: feedback
  if(process_it){
    process_xbee_request();
    process_it=false;
  }


  delay(1000);
}


