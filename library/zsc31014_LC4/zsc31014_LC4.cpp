#include "zsc31014_LC4.h"



LC4::LC4(uint8_t _en_p, uint8_t _int_p, uint8_t _addr){
    EN_P = _en_p;
    INT_P = _int_p;
    i2c_addr = _addr;
    pinMode(EN_P,OUTPUT);
    pinMode(INT_P,INPUT);
    digitalWrite(EN_P,HIGH);
}

uint8_t LC4::loadcell_setup(float _max, float _rated)
{
    max_LC = (float)_max;
    rated_output = (float)_rated;
    eeprom_update();
    return 1;
}

void LC4::dev_i2c_read(uint8_t *read_buf)
{
    Wire.requestFrom(i2c_addr, (uint8_t)4);
    for (uint8_t i = 0; Wire.available() && (i < 4); i++)
    {
        read_buf[i] = Wire.read();
    }
}

void LC4::read_raw(uint32_t *rx_data)
{
    uint8_t rx_buf[4];
    uint32_t tmp;
    dev_i2c_read(rx_buf);
    tmp = rx_buf[0];
    tmp <<= 8;
    tmp |= rx_buf[1];
    tmp <<= 8;
    tmp |= rx_buf[2];
    tmp <<= 8;
    tmp |= rx_buf[3];

    *rx_data = tmp;
}

uint8_t LC4::read_data(uint16_t *bridge_data, int16_t *temperature_data)
{
    uint32_t raw_data;
    uint8_t status_data;
    uint16_t temp_data;

    read_raw(&raw_data);

    status_data = raw_data >> 30;
    status_data &= LOADCELL4_STATUS_BIT_MASK;

    temp_data = raw_data >> 16;
    temp_data &= LOADCELL4_BRIDGE_RES;
    *bridge_data = temp_data;

    temp_data = raw_data;
    temp_data >>= 5;
    *temperature_data = temp_data;

    return status_data;
}

uint16_t LC4::read_bridge_data()
{
    uint32_t raw_data;
    uint8_t status_data;
    uint16_t bridge_data;

    status_data = LOADCELL4_STATUS_ERROR;

    while (status_data != LOADCELL4_STATUS_NORMAL)
    {
        read_raw(&raw_data);
        dev_measure_delay();

        status_data = raw_data >> 30;
        status_data &= LOADCELL4_STATUS_BIT_MASK;

        bridge_data = raw_data >> 16;
        bridge_data &= LOADCELL4_BRIDGE_RES;
    }

    return bridge_data;
}

// get loadcell value considering tare value, calibration value
float LC4::loadcell_read()
{
    uint16_t results;
    float tare_val;
    float weight_val;
    tare_val = tare;
    while (!digitalRead(INT_P))
        ;
    results = read_bridge_data();
    // Serial.println(results);
    weight_val = (float)results - tare_val;
    //weight_val = weight_val * max_LC / ((rated_output/1000.00) * (float)amp_gain) / 16384.00;
    weight_val = weight_val * max_LC / (rated_output * (float)amp_gain) * 125.00 / 2048.00;

    return weight_val;
}

void LC4::loadcell_tare()
{
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;

    sum_val = 0;

    for (n_cnt = 0; n_cnt < 100; n_cnt++)
    {
        // while (digitalRead(INT_P) == LOW)
        //     ;
        results = read_bridge_data();
        sum_val += results;
        dev_measure_delay();
        // Serial.println(results);
        delay(1);
    }

    average_val = (float)sum_val;
    average_val /= LOADCELL4_NUMB_OF_SUM_AVG_100;

    tare = average_val;
    Serial.println(tare);
}

// CM mode
bool LC4::CM_mode_on(){
    uint8_t status_data = 0;
    uint8_t read_buf[3] = {0, 0, 0};
    int _state_error = 0;
    while (status_data != LOADCELL4_EEPROM_STATUS_READ_READY)
    {
        digitalWrite(EN_P, LOW);
        dev_hw_reset_delay();
        digitalWrite(EN_P, HIGH);
        Wire.beginTransmission(i2c_addr);
        Wire.write(0xA0);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        dev_measure_delay();
        Wire.beginTransmission(i2c_addr);
        Wire.write(read_buf, (uint8_t)3);
        Wire.endTransmission();
        dev_measure_delay();
        Wire.requestFrom(i2c_addr, (uint8_t)3);
        delay(1);
        for (uint8_t i = 0; Wire.available() && (i < 3); i++)
        {
            read_buf[i] = Wire.read();
        }
        status_data = read_buf[0];
        if (status_data == LOADCELL4_EEPROM_STATUS_READ_READY)
            Serial.println("CM mode on");
        else
        {
            Serial.println("CM mode does not enter");
            Serial.println(status_data);
        }
        dev_measure_delay();
    }
    return true;
}

void LC4::CM_mode_off(){
    Wire.beginTransmission(i2c_addr);
    Wire.write(0x80);
    Wire.write(0x00);
    Wire.write(0x00);
    bool x = Wire.endTransmission();
    dev_CM_write_delay();
    delay(100);
    while (!digitalRead(INT_P))
        ;
    if (!x)
        Serial.println("CM mode off");
    else
        Serial.println("CM mode does not out");
}

void LC4::write_eeprom(uint8_t cmd_byte, uint16_t data_word)
{
    uint8_t eeprom_word = cmd_byte;
    eeprom_word |= LOADCELL4_EEPROM_WRITE_CMD;
    // dev_i2c_write(ctx, eeprom_word, data_word);
    Wire.beginTransmission(i2c_addr);
    Wire.write( eeprom_word );
    Wire.write( data_word >> 8 );
    Wire.write( data_word & 0xFF );
    Wire.endTransmission();
    dev_CM_write_delay();
    while(!digitalRead(INT_P))
        ;
}

uint16_t LC4::read_eeprom(uint8_t cmd_byte)
{
    uint8_t status_data = 0;
    uint16_t data_word;
    uint8_t read_buf[3] = {0, 0, 0};
    int _state_error = 0;
    read_buf[0] = cmd_byte;

    while (status_data != LOADCELL4_EEPROM_STATUS_READ_READY)
    {
        Wire.beginTransmission(i2c_addr);
        Wire.write(read_buf, (uint8_t)3);
        Wire.endTransmission();
        dev_measure_delay();
        Wire.requestFrom(i2c_addr, (uint8_t)3);
        delay(1);
        for (uint8_t i = 0; Wire.available() && (i < 3); i++)
        {
            read_buf[i] = Wire.read();
        }
        status_data = read_buf[0];
        data_word = read_buf[1];
        data_word <<= 8;
        data_word |= read_buf[2];

        dev_measure_delay();
    }
    return data_word;
}
// private function

/**
 * @brief Measurement delay function.
 * @details The function performs the measurement delay of 10micro seconds.
 */
void LC4::dev_measure_delay(void)
{
    delayMicroseconds(12);
}

void LC4::dev_reset_delay(void)
{
    delay(20);
}

void LC4::dev_hw_reset()
{
    digitalWrite(EN_P, LOW);
    dev_hw_reset_delay();
    digitalWrite(EN_P, HIGH);
    dev_reset_delay();
}

void LC4::dev_hw_reset_delay(){
    delay(1000);
}

void LC4::dev_CM_write_delay()
{
    delay(20);
}

void LC4::eeprom_value_assign(uint8_t _addr, uint16_t _val){
    if (_addr == LOADCELL4_EEPROM_CONFIG_1)
    {
        // clock setting
        uint8_t det_val = _val & LOADCELL4_EEPROM_CLKSPEED;
        if(det_val) clkspeed = 1;
        else clkspeed = 4;
        // operation mode setting
        det_val = _val & LOADCELL4_EEPROM_OPMODE;
        if(det_val) op_mode = 'S';
        else
            op_mode = 'U';
        //update rate setting
        det_val = (_val & LOADCELL4_EEPROM_UPRATE) >> (int)(LC4_UPRATE & 0x0F);
        if(det_val == 0x00){
            if(clkspeed == 1) 
                u_rate = 1.60;
            else
                u_rate = 0.50;
        }
        else if (det_val == 0x01)
        {
            if (clkspeed == 1)
                u_rate = 5.00;
            else
                u_rate = 1.50;
        }
        else if (det_val == 0x10)
        {
            if (clkspeed == 1)
                u_rate = 25.00;
            else
                u_rate = 6.50;
        }
        else
        {
            if (clkspeed == 1)
                u_rate = 125.00;
            else
                u_rate = 32.00;
        }
    }
    else if(_addr == LOADCELL4_EEPROM_CONFIG_2){
        uint8_t det_val = (_val & LOADCELL4_EEPROM_ADDR) >> (int)(LC4_ADDR&0x0F);
        i2c_addr = det_val;
    }
    else if (_addr == LOADCELL4_EEPROM_BRIDGE_CONFIG)
    {
        uint8_t det_val = (_val & LOADCELL4_EEPROM_GAIN) >> (int)(LC4_GAIN & 0x0F);
        switch (det_val)
        {
            case 0x00: amp_gain = 1.50; break;
            case 0x01: amp_gain = 3.00; break;
            case 0x02: amp_gain = 6.00; break;
            case 0x03: amp_gain = 12.00; break;
            case 0x04: amp_gain = 24.00; break;
            case 0x05: amp_gain = 48.00; break;
            case 0x06: amp_gain = 96.00; break;
            case 0x07: amp_gain = 192.00; break;
            default: amp_gain = 192.00; break;
        }
    }
}

void LC4::eeprom_whole_read()
{
    for (uint8_t i = 0x00; i < 0x14; ++i)
    {
        uint16_t tmp = read_eeprom(i);
        EEP_val[(int)i] = tmp;
        eeprom_value_assign(i, tmp);
        Serial.print(i);
        Serial.print("  :       ");
        Serial.println(EEP_val[(int)i]);
    }
    Serial.print("clkspeed:    ");
    Serial.println(clkspeed);
    Serial.print("Operation Mode:    ");
    Serial.println(op_mode);
    Serial.print("update_rate:    ");
    Serial.println(u_rate);
    Serial.print("amp gain:    ");
    Serial.println(amp_gain);
    Serial.print("LC4 I2C Address:    ");
    Serial.println(i2c_addr);
}

void LC4::eeprom_update()
{
    CM_mode_on();
    eeprom_whole_read();
    CM_mode_off();
}

void LC4::amp_setup(int args, ...)
{
    CM_mode_on();
    if (EEP_val[0])
    {
        va_list ap;
        va_start(ap, args);
        for (int i = 0; i < args; i += 2)
        {
            uint8_t det_val = (uint8_t)va_arg(ap, int);
            uint8_t setting_val = (uint8_t)va_arg(ap, int);
            Serial.println(det_val);
            Serial.println(setting_val);
            if ((det_val & 0xF0) == 0x10)
            {
                if ((det_val & 0x0F) == (LC4_CLK & 0x0F))
                {
                    if (setting_val < 0x02)
                        EEP_val[(int)(det_val >> 4)] = (EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_CLKSPEED)) | ((uint16_t)(setting_val << ((int)(LC4_CLK & 0x0F))));
                }
                else if ((det_val & 0x0F) == (LC4_OPMODE & 0x0F))
                {
                    if (setting_val < 0x02)
                        EEP_val[(int)(det_val >> 4)] = (EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_OPMODE)) | ((uint16_t)(setting_val << ((int)(LC4_OPMODE & 0x0F))));
                }
                else if ((det_val & 0x0F) == (LC4_UPRATE & 0x0F))
                {
                    if (setting_val < 0x04)
                        EEP_val[(int)(det_val >> 4)] = (EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_UPRATE)) | ((uint16_t)(setting_val << ((int)(LC4_UPRATE & 0x0F))));
                }
            }
            else if ((det_val & 0xF0) == 0x20)
            {
                if ((det_val & 0x0F) == (LC4_ADDR & 0x0F))
                {
                    if (setting_val < 0x7F)
                        // EEP_val[(int)(det_val >> 4)] = (EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_ADDR)) | ((uint16_t)(setting_val << ((int)(LC4_ADDR & 0x0F))));
                        Serial.println((EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_ADDR)) | ((uint16_t)(setting_val << ((int)(LC4_ADDR & 0x0F)))));
                }
            }
            else if ((det_val & 0xF0) == 0xF0)
            {
                if ((det_val & 0x0F) == (LC4_GAIN & 0x0F))
                {
                    if (setting_val < 0x08)
                        EEP_val[(int)(det_val >> 4)] = (EEP_val[(int)(det_val >> 4)] & (0xFFFF - (uint16_t)LOADCELL4_EEPROM_GAIN)) | ((uint16_t)(setting_val << ((int)(LC4_GAIN & 0x0F))));
                }
            }
        }
        va_end(ap);
        write_eeprom(0x01, EEP_val[1]);
        write_eeprom(0x0F, EEP_val[15]);
        eeprom_whole_read();
        CM_mode_off();
    }
}