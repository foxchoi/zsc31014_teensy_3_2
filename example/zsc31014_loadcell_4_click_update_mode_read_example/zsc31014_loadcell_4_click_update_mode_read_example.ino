/* zsc31014 loadcell 4 click update mode read example*/


#include "zsc31014_LC4.h"
const int lc4_en = 3; //Enabling pin number
const int lc4_int = 2;  // interrupt pin number
LC4 LC4_test(lc4_en, lc4_int); // define loadcell class

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(); //i2c communication
  LC_test.loadcell_setup(200.00, 1.416); // loadcell_setup(maximun load, rated output mV/V)
  LC_test.loadcell_tare(); // zero value calculate


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(LC_test.loadcell_read()); //read loadcell data. Update mode
  delay(10);

}
