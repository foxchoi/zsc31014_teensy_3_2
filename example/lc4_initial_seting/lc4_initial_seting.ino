//#include <hsh_lc4.h>
#include "Wire.h"

const int lc4_en = 3;
const int lc4_int = 2;

volatile float weight_val = 0.0f;
const uint32_t initial_average_value = 2048;
static uint16_t lc4_value;
uint8_t addr = 0x01;
uint8_t read_buf[4];
uint8_t write_buf[3];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
    pinMode(lc4_en,OUTPUT);
    pinMode(lc4_int,INPUT);
    digitalWrite(lc4_en,HIGH);
    /*digitalWrite(lc4_en,HIGH);
    delay(1000);
//    digitalWrite(lc4_en,HIGH);
      Serial.print("  addr: ");
    digitalWrite(lc4_en,LOW);
    delay(3000);
      Serial.print(0x28);
      Serial.print("  val: ");
    digitalWrite(lc4_en,HIGH);
//     cm mode
      Wire.beginTransmission(0x28);
      Wire.write(0xA0);
      Wire.write(0x00);
      Wire.write(0x00);
      int x = Wire.endTransmission(1);
      Serial.println(x,DEC);
      delay(10);
      Serial.println("Write EEPROM  ");
      Wire.beginTransmission(0x28);
      write_buf[0] = 0x01;
      write_buf[1] = 0x00;
      write_buf[2] = 0x00;
      Wire.write(write_buf,3);
      int y = Wire.endTransmission(1);
      Serial.println(y,DEC);
      Wire.requestFrom(0x28, 3);
      delay(1000);
      Serial.println("Read Result: ");
    for(uint8_t i = 0; Wire.available() && (i < 4); i++){
        read_buf[i] = Wire.read();
        Serial.println(read_buf[i]);
    }
      Wire.beginTransmission(0x28);
      write_buf[0] = 0x80;
      write_buf[1] = 0x00;
      write_buf[2] = 0x00;
      Wire.write(write_buf,3);
      Wire.endTransmission(1);
      delay(1000);*/
    //digitalWrite(lc4_en,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("new");
   Wire.requestFrom( 0x28 , 4 ); 
    delay(10);
    Serial.println("Data:   ");
    for(uint8_t i = 0; Wire.available() && (i < 4); i++){
        read_buf[i] = Wire.read();
        Serial.println(read_buf[i]);
    }
Serial.println(digitalRead(lc4_int));
  //Serial.println(lc4_value);
  delay(10);
}
