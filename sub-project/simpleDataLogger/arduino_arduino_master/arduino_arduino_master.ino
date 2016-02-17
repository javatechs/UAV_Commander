// Written by Nick Gammon
// February 2011


#include <SPI.h>

char rx_buffer[10];
int8_t rx_index=0;
int8_t loop_counter=0;

 byte transferAndWait (const byte what)
{
  byte a = SPI.transfer (what);
  delayMicroseconds (50);
  return a;
} // end of transferAndWait



void setup (void)
{

  Serial.begin(9600);
  digitalWrite(SS, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
}  // end of setup


void loop (void)
{

  char c;
  uint16_t raw_read=0;
  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  // send test string
  switch(loop_counter){
    case 0:
        for (const char * p = "STATUS\0" ; c = *p; p++){
          SPI.transfer (c);
        }
        loop_counter++;
        break; 
    case 1:
        for (const char * p = "TEMPERATURE\0" ; c = *p; p++){
          SPI.transfer (c);
        }
        loop_counter++;
        break; 
     case 2:
        for (const char * p = "HUMIDITY\0" ; c = *p; p++){
          SPI.transfer (c);
        }
        loop_counter++;
        break; 
     case 3:
        for (const char * p = "SOIL\0" ; c = *p; p++){
          SPI.transfer (c);
        }
        loop_counter=0;
        break; 
  }


  // disable Slave Select
  digitalWrite(SS, HIGH);

  delay(100);

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  c='H';
  //Read 0x7f
  c=transferAndWait(0x0);
  //Read payload Length
  c=transferAndWait(0x0);
  int8_t msg_len=c;
  raw_read=0;
  for (int8_t i=0;i<msg_len;i++){
    c=transferAndWait(0x0);
    //Serial.print(c,HEX);
    //Serial.print(' ');
    if(loop_counter!=0){
      raw_read=(raw_read<<8)&0xff00 | (uint8_t)c&0xff;
    }
    
  }

  switch(loop_counter){
    
    case 2:
      Serial.print("Now TEMPERATURE is: ");
      Serial.print((double)raw_read/10);
      break;
    case 3:
      Serial.print("Now HUMIDITY is: ");
      Serial.print((double)raw_read/10);
      break;
    case 0:
      Serial.print("Now SOIL is: ");
      Serial.print((double)raw_read/10);
      break;
  }

  
  // disable Slave Select
  digitalWrite(SS, HIGH);
  Serial.println("");

  delay (2000);  // 1 seconds delay 
}  // end of loop

