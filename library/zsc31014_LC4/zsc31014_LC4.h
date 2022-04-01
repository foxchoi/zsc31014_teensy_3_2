/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/****************************************************************************
** Modified by Seongheon Hong, Hyungmin Choi - BRL (March 27, 2022)
** Compatible with Arduino IDE & Wire library
****************************************************************************/

// #ifndef LOADCELL4_H
// #define LOADCELL4_H

#ifdef __cplusplus
// extern "C"{
#endif

// #ifndef Arduino_H
// #define Arduino_H

#include "Wire.h"
#include "Arduino.h"

/**
 * @brief Load Cell 4 description setting.
 * @details Specified setting for description of Load Cell 4 Click driver.
 */
#define LOADCELL4_GET_RESULT_ERROR 0
#define LOADCELL4_GET_RESULT_OK 1
#define LOADCELL4_DATA_NO_DATA 0
#define LOADCELL4_DATA_OK 1
#define LOADCELL4_WEIGHT_100G 100
#define LOADCELL4_WEIGHT_500G 500
#define LOADCELL4_WEIGHT_1000G 1000
#define LOADCELL4_WEIGHT_5000G 5000
#define LOADCELL4_WEIGHT_10000G 10000
#define LOADCELL4_DEFAULT_WEIGHT_SCALE_COEFFICIENT 0.088495575221

/**
 * @brief Load Cell 4 eeprom setting.
 * @details Specified eeprom setting of Load Cell 4 Click driver.
 */
#define LOADCELL4_EEPROM_ID_0 0x00
#define LOADCELL4_EEPROM_CONFIG_1 0x01
#define LOADCELL4_EEPROM_CONFIG_2 0x02
#define LOADCELL4_EEPROM_OFFSET_BRIDGE 0x03
#define LOADCELL4_EEPROM_GAIN_BRIDGE 0x04
#define LOADCELL4_EEPROM_TEMP_COF_GAIN 0x05
#define LOADCELL4_EEPROM_TEMP_COF_OFFSET 0x06
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_OFFSET 0x07
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_GAIN 0x08
#define LOADCELL4_EEPROM_2ND_ORDER_BRIDGE 0x09
#define LOADCELL4_EEPROM_OFFSET_TEMP 0x0A
#define LOADCELL4_EEPROM_GAINT_TEMP 0x0B
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP 0x0C
#define LOADCELL4_EEPROM_TEMP_SETL 0x0D
#define LOADCELL4_EEPROM_ID_1 0x0E
#define LOADCELL4_EEPROM_BRIDGE_CONFIG 0x0F
#define LOADCELL4_EEPROM_TEMP_CONFIG 0x10
#define LOADCELL4_EEPROM_OSC_TRIM 0x11
#define LOADCELL4_EEPROM_SIGNATURE 0x12
#define LOADCELL4_EEPROM_ID_2 0x13
#define LOADCELL4_EEPROM_WRITE_CMD 0x40
#define LOADCELL4_EEPROM_STATUS_READ_READY 0x5A
#define LOADCELL4_EEPROM_CLKSPEED 0x08
#define LOADCELL4_EEPROM_OPMODE 0x20
#define LOADCELL4_EEPROM_UPRATE 0xC0
#define LOADCELL4_EEPROM_GAIN 0x70
#define LOADCELL4_EEPROM_ADDR 0x3F8
// loadcell initial setting
#define LC4_GAIN 0xF4
#define LC4_CLK 0x13
#define LC4_OPMODE 0x15
#define LC4_UPRATE 0x16
#define LC4_ADDR 0x23
/**
 * @brief Load Cell 4 status.
 * @details Status settings of Load Cell 4 Click driver.
 */
#define LOADCELL4_STATUS_NORMAL 0x00
#define LOADCELL4_STATUS_CMD_MODE 0x01
#define LOADCELL4_STATUS_STALE_DATA 0x02
#define LOADCELL4_STATUS_ERROR 0x03
#define LOADCELL4_STATUS_BIT_MASK 0x03
#define LOADCELL4_BRIDGE_RES 0x3FFF

/**
 * @brief Load Cell 4 dummy.
 * @details Dummy data for Load Cell 4 Click driver.
 */
#define LOADCELL4_DUMMY 0x0000

/**
 * @brief Load Cell 4 power setting.
 * @details Specified power setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_PWR_OFF 0x00
#define LOADCELL4_PWR_ON 0x01

/**
 * @brief Load Cell 4 zero weight.
 * @details Zero weight setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_WEIGHT_ZERO 0.000

/**
 * @brief Load Cell 4 number of average values.
 * @details Number of average values used for calibrating
 * for Load Cell 4 Click driver.
 */
#define LOADCELL4_NUMB_OF_SUM_AVG_20 20.00
#define LOADCELL4_NUMB_OF_SUM_AVG_100 100.0

/**
 * @brief Load Cell 4 command mode.
 * @details Command mode setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_CMD_MODE_START 0xA0
#define LOADCELL4_CMD_MODE_STOP 0x80

/**
 * @brief Load Cell 4 device address setting.
 * @details Specified setting for device slave address selection of
 * Load Cell 4 Click driver.
 */
#define LOADCELL4_SET_DEV_ADDR 0x28


/**
 * @brief  Loadcell 4 click class
 * @details ZSC31014 Loadcell 4 Click class objects that would be declared
 */

class LC4
{
    private:
        // I2C related value
        // uint32_t i2c_speed = 100000; // i2c communication speed, default 100kHz
        uint8_t i2c_addr = 0x28;  // LC4 slave address, default 0x28 when no address has been set
        // Loadcell related value
        float max_LC; // maximum measurable value of loadcell
        float rated_output; // rated output of loadcell in the datashett
        // pin number connected to LC4
        uint8_t scl;
        uint8_t sda;
        uint8_t EN_P; // pin connected to enable pin of LC4
        uint8_t INT_P; // pin connected to interrupt pin of LC4
        // EEPROM value
        uint16_t EEP_val[20]={0,};
        float amp_gain = 192.00; // gain
        int clkspeed = 4; // clock speed, 4: 4MHz, 1: 1MHz
        char op_mode = 'U'; // operation mode, 'U': Update mode, 'S': Sleep mode
        float u_rate = 0.50; // update rate. unit: ms
        // Loadcell calibration value
        float tare = 0.00;                   // no load value
        // private function
        void dev_measure_delay(void); // delay function
        void dev_reset_delay(void);   // delay when doing hardware reset, allocate reset_delay function
        // static void dev_measure_delay(void); // delay when measurement occur
        void dev_hw_reset_delay();       // delay when reset hardware by tunr off and on using EN pin when going to CM mode
        void dev_CM_write_delay();       // delay when write  in CM mode
        void dev_hw_reset();             // reset hardware by tunr off and on using EN pin
        void eeprom_value_assign(uint8_t _addr, uint16_t _val); // assign value from eeprom data
        void eeprom_update();
        void eeprom_whole_read();

    public:
        LC4(uint8_t _en_p, uint8_t _int_p, uint8_t _addr = 0x28); // initiate LC4
        uint8_t loadcell_setup(float _max, float _rated);         // setting loadcecll rated value
        void loadcell_tare();  // zero set data
        void dev_i2c_read(uint8_t *read_buf); // read data from i2c (4byte)
        void read_raw(uint32_t *rx_data);            // integrate raw data into 32bit data into rx_data
        uint8_t read_data(uint16_t *bridge_data, int16_t *temperature_data); //allocate data into bridge (loadcell data), and temp. data(temperature). return:status data
        uint16_t read_bridge_data();                                         // return bridge data
        float loadcell_read();                                               // read loadcell data
        // CM mode (read/write EEPROME)
        bool CM_mode_on();
        void CM_mode_off();
        void write_eeprom(uint8_t cmd_byte, uint16_t data_word);
        uint16_t read_eeprom(uint8_t cmd_byte);
        void amp_setup(int args, ...);
};


// void loadcell4_cfg_setup(loadcell4_cfg_t *cfg, uint8_t intpin, uint8_t enpin, uint8_t addr);
// uint8_t loadcell4_init(loadcell4_t *ctx, loadcell4_cfg_t *cfg, float max_load, float rated_output);
// uint8_t loadcell4_default_cfg(loadcell4_t *ctx);
// uint8_t loadcell4_generic_write(loadcell4_t *ctx, uint8_t reg, uint8_t *tx_buf, uint8_t tx_len);
// uint8_t loadcell4_generic_read(loadcell4_t *ctx, uint8_t reg, uint8_t *rx_buf, uint8_t rx_len);
// void loadcell4_read_raw(loadcell4_t *ctx, uint32_t *rx_data);
// uint8_t loadcell4_read_data(loadcell4_t *ctx, uint16_t *bridge_data, int16_t *temperature_data);
// uint16_t loadcell4_read_bridge_data(loadcell4_t *ctx);
// void loadcell4_start_cmd_mode(loadcell4_t *ctx);
// void loadcell4_end_cmd_mode(loadcell4_t *ctx);
// void loadcell4_power_dev(loadcell4_t *ctx, uint8_t power_state);
// void loadcell4_write_eeprom(loadcell4_t *ctx, uint8_t cmd_byte, uint16_t data_word);
// uint16_t loadcell4_read_eeprom(loadcell4_t *ctx, uint8_t cmd_byte);
// uint8_t loadcell4_get_int(loadcell4_t *ctx);
// void loadcell4_tare(loadcell4_t *ctx, loadcell4_data_t *cell_data);
// void loadcell4_tare_skip(loadcell4_t *ctx, loadcell4_data_t *cell_data);
// uint8_t loadcell4_calibration(loadcell4_t *ctx, uint16_t cal_val, loadcell4_data_t *cell_data);
// uint8_t loadcell4_calibration_skip(loadcell4_t *ctx, uint16_t cal_val, loadcell4_data_t *cell_data);
// float loadcell4_get_weight(loadcell4_t *ctx, loadcell4_data_t *cell_data);
// float loadcell4_gw_hm(loadcell4_t *ctx, loadcell4_data_t *cell_data);
// float loadcell4_get_raw_weight_data(loadcell4_t *ctx, loadcell4_data_t *cell_data, uint8_t shotCounts);

#ifdef __cplusplus
//}
#endif
// #endif // LOADCELL4_H
