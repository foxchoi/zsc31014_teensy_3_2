/* zsc31014 loadcell 4 click eeprom write example*/

#include "zsc31014_LC4.h"
const int lc4_en = 3; //Enabling pin number
const int lc4_int = 2;  // interrupt pin number
LC4 LC4_test(lc4_en, lc4_int); // define loadcell class

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin()
  LC_test.loadcell_setup(200.00, 1.416);// loadcell_setup(maximun load, rated output mV/V)
  LC_test.amp_setup(8,LC4_GAIN,0x07,LC4_CLK,0x00,LC4_OPMODE,0x00,LC4_UPRATE,0x00);
  /* setup the loadcell 4 click amp
   *  amp_setup(number of data, which value ,(uint8_t)value,...)
   *  LC4_GAIN: gain of amp. 0x00~0x07
   *  LC4_CLK: clock frequency of amp. 0x00: 4MHz, 0x01: 1MHz
   *  LC4_OPMODE: operation mode. 0x00: update mode, 0x01: sleep mode
   *  LC4_UPRATE: update rate. 0x00~0x03
   *  reference: datasheet 39~42 page
  */

}

void loop() {
  // put your main code here, to run repeatedly:

}
