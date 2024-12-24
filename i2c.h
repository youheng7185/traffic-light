#ifndef I2C_H
#define I2C_H

bool i2c_write_addr8_data8(uint8_t address, uint8_t data);
bool i2c_read_addr8_data8(uint8_t address, uint8_t &data);
bool i2c_read_addr8_data16(uint8_t address, uint16_t &data);
void i2c_set_slave_address(uint8_t addr);

#endif /* I2C_H */
