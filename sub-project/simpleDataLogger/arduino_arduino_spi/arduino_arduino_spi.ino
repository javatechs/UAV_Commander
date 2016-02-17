// IMPORTANT!
// SPI MESSAGE SENDING BACK BY LSB!!!
#include <SPI.h>
#include <SoftwareSerial.h>

//For Xbee
SoftwareSerial mySerial(8, 9); // RX, TX
char xbee_rx_buffer[10];

//For SPI
enum SPI_command_t{
  NO,
  STATUS,
  TEMPERATURE,
  HUMIDITY,
  SOIL
};

enum SPI_status_t{
  SPI_IDLE,
  SPI_SENDING,
  SPI_RECEIVING
};


double temp=24.5;
double humi=30.2;
double soil=2.3;
char rx_buffer[20];
volatile int8_t rx_index=0;
volatile char tx_buffer[20];
volatile int8_t tx_index=0;
volatile int8_t process_it=0;
volatile SPI_command_t SPI_command=NO;
volatile SPI_status_t SPI_status=SPI_RECEIVING;
volatile int8_t tx_length=0;

void mystrcpy(char* tx, volatile char* rx, int count){
  int i=0;
  for (i=0;i<count;i++){
    tx[i]=rx[i];
  }
}

void tx_str_serialize(char* str, volatile char* tx, volatile int8_t* len){
  int8_t payload_len = ((String)str).length();
  int8_t temp_len = payload_len+3;
  int8_t i=0;
  tx[0]=0x7F;   //start char
  tx[1]=payload_len;
  for(i=0;i<payload_len;i++){
    tx[i+2]=str[i];
  }
  *len = temp_len;
  //checksum as 0xff first
  tx[temp_len-1]=0xff;
}

void tx_data_serialize(int16_t data, volatile char* tx, volatile int8_t* len){
  int8_t payload_len = 2;
  int8_t temp_len = payload_len+3;
  int8_t i=0;
  tx[0]=0x7F;   //start char
  tx[1]=2;
  tx[2]=(data>>8)&0xff;
  tx[3]=(data&0xff);
  *len = temp_len;
  //checksum as 0xff first
  tx[temp_len-1]=0xff;
  
}

void read_command(){
  if(SPI_status==SPI_SENDING){
     digitalWrite(7,LOW);
      //send over. Dont do sending complete check now.
      SPI_status=SPI_RECEIVING;
      
      rx_index=0;
      tx_index=0;
      tx_length=0;
      Serial.println("Sending finished");
      return;
    }
  digitalWrite(7,HIGH);
  //read command after reading
  if(strcmp(rx_buffer, "STATUS")==0){
    Serial.println("Read Command: STATUS");
    SPI_command=STATUS;
    tx_str_serialize("OK", tx_buffer, &tx_length);
    SPI_status = SPI_SENDING;
    memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
    SPDR=tx_buffer[0];
    tx_index=1;
    rx_index=0;
    return;
  }

  if(strcmp(rx_buffer, "TEMPERATURE")==0){
    Serial.println("Read Command: TEMPERATURE");
    SPI_command=TEMPERATURE;
    int16_t temp16=(int16_t)(temp*10);
    tx_data_serialize(temp16, tx_buffer, &tx_length);
    SPI_status = SPI_SENDING;
    memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
    SPDR=tx_buffer[0];
    tx_index=1;
    rx_index=0;
    return;
  }

  if(strcmp(rx_buffer, "HUMIDITY")==0){
    SPI_command=HUMIDITY;
    int16_t humi16=(int16_t)(humi*10);
    tx_data_serialize(humi16, tx_buffer, &tx_length);
    SPI_status = SPI_SENDING;
    memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
    SPDR=tx_buffer[0];
    tx_index=1;
    rx_index=0;
    return;
  }

  if(strcmp(rx_buffer, "SOIL")==0){
    SPI_command=SOIL;
    int16_t soil16=(int16_t)(soil*10);
    tx_data_serialize(soil16, tx_buffer, &tx_length);
    SPI_status = SPI_SENDING;
    memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
    SPDR=tx_buffer[0];
    tx_index=1;
    rx_index=0;
    return;
  }

  //no command, and reading is ending
  Serial.println("NO Command");
  SPI_status = SPI_RECEIVING;
  memset(rx_buffer, 0, sizeof(rx_buffer)/sizeof(rx_buffer[0]));
  rx_index=0;
}

//For XBEE
void send_command(char* command_name){
  mySerial.print(command_name);
  mySerial.print('\0');
}

void get_value(double* data){
  uint8_t i=0;
  int16_t temp_data;
  while(mySerial.available()){
    xbee_rx_buffer[i]=(char)(mySerial.read());
    i++;
  }
  i=0;
  for(i=0;i<10;i++){
    if(xbee_rx_buffer[i]==0x7f){
      temp_data = (xbee_rx_buffer[i+2]<<8)&0xff00 | (xbee_rx_buffer[i+3]&0xff);
    }else{
      i++;
    }
  }
  *data = (double)temp_data/10.0;
}


void setup() {
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  pinMode(7,OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

 

  attachInterrupt(digitalPinToInterrupt(2), read_command, RISING);

  Serial.begin(9600);
  mySerial.begin(9600);
}

ISR(SPI_STC_vect){
  byte rx = SPDR;

  if (SPI_status == SPI_RECEIVING){
    //Serial.print((char)rx);
    rx_buffer[rx_index]=rx;
    rx_index+=1;
  }else if (SPI_status==SPI_SENDING){
    if(tx_index>=(tx_length-1)){
      //sending end
      SPDR=0x0;
    }else{
      SPDR=tx_buffer[tx_index];
      //Serial.print(SPDR,HEX);
      //Serial.print(' ');
      tx_index++;
      return;
    }
  }
  
}

void loop() {
  Serial.println("Reading: ");
  // put your main code here, to run repeatedly:
  //every 5s it pull data
  send_command("TEMPERATURE");
  delay(2000);
  get_value(&temp);
  send_command("HUMIDITY");
  delay(2000);
  get_value(&humi);
  send_command("SOIL");
  delay(2000);
  get_value(&soil);

  Serial.print("TEMP IS: ");
  Serial.print(temp);
  Serial.print('\t');

  Serial.print("HUMI IS: ");
  Serial.print(humi);
  Serial.print('\t');

  Serial.print("SOIL IS: ");
  Serial.print(soil);
  Serial.print('\t');
  Serial.print('\n');
  delay(2000);
  
}
