#include "robonaut_config.h"

void Start_Gyroscope_Task(void const * argument);
void BNO055_delay_msek(u32 msek);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
int8_t I2C_routine(void);
