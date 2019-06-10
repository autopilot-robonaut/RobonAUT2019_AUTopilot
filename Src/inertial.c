#include "inertial.h"

#define	I2C_BUFFER_LEN 16
#define I2C0 5
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)

#define I2CTIMEOUT 100



struct bno055_t bno055;



/* Variable used to return value of
communication routine*/
s32 comres = BNO055_ERROR;
/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;

/*******************read euler converted data*******************/
/* variable used to read the euler h data output
as degree or radians*/
double d_euler_data_h = BNO055_INIT_VALUE;
/* variable used to read the euler r data output
as degree or radians*/
double d_euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data output
as degree or radians*/
double d_euler_data_p = BNO055_INIT_VALUE;
/* structure used to read the euler hrp data output
as as degree or radians */
struct bno055_euler_double_t d_euler_hpr;
uint8_t avarage_ind=0;
float sum_yaw;

struct bno055_sic_matrix_t sic_matrix_bosch;
struct bno055_accel_offset_t accel_offset_bosch;
struct bno055_mag_offset_t mag_offset_bosch;
struct bno055_gyro_offset_t gyro_offset_bosch;

float yaw,yaw_ideiglenes;

	/*********read raw accel data***********/
	/* variable used to read the accel x data */
	s16 accel_datax = BNO055_INIT_VALUE;
	 /* variable used to read the accel y data */
	s16 accel_datay = BNO055_INIT_VALUE;
	/* variable used to read the accel z data */
	s16 accel_dataz = BNO055_INIT_VALUE;
	/* variable used to read the accel xyz data */
	struct bno055_accel_t accel_xyz;

u8 chip_id;
u16 sw_id_u8;
u8 accel_rev_id_u8;
u8 accel_calib_u8;
u8 operation_mode_u8;
u8 power_mode_u8;
u8 mag_calib_u8,gyro_calib_u8,system_calib_u8;

void Start_Gyroscope_Task(void const * argument)
{
	osDelay(1000);
	/*
	 * Initalization code
	 */
	I2C_routine();
	
	comres = bno055_init(&bno055);
	
//	comres += bno055_read_chip_id(&chip_id);
//	
//	comres += bno055_read_sw_rev_id(&sw_id_u8);
//	
//	comres += bno055_read_accel_rev_id(&accel_rev_id_u8);
	
	//comres += bno055_read_sic_matrix(&sic_matrix_bosch);
//	sic_matrix_bosch.sic_0 = 16384;
//	sic_matrix_bosch.sic_1 = 0;
//	sic_matrix_bosch.sic_2 = 0;
//	sic_matrix_bosch.sic_3 = 0;
//	sic_matrix_bosch.sic_4 = 16384;
//	sic_matrix_bosch.sic_5 = 0;
//	sic_matrix_bosch.sic_6 = 0;
//	sic_matrix_bosch.sic_7 = 0;
//	sic_matrix_bosch.sic_8 = 16384;
//	comres +=  bno055_write_sic_matrix(&sic_matrix_bosch);
	
	//comres += bno055_read_accel_offset(&accel_offset_bosch);
	accel_offset_bosch.x = 69;
	accel_offset_bosch.y = -12;
	accel_offset_bosch.z = -9;
	accel_offset_bosch.r = 1000;
	comres += bno055_write_accel_offset(&accel_offset_bosch);
	
	//comres += bno055_read_mag_offset(&mag_offset_bosch);
//	mag_offset_bosch.x = 106;
//	mag_offset_bosch.y = 93;
//	mag_offset_bosch.z = 262;
//	mag_offset_bosch.r = 712;
//	comres += bno055_write_mag_offset(&mag_offset_bosch);
	
	//comres += bno055_read_gyro_offset(&gyro_offset_bosch);
	gyro_offset_bosch.x = -1;
	gyro_offset_bosch.y = -2;
	gyro_offset_bosch.z = 1;
	comres += bno055_write_gyro_offset(&gyro_offset_bosch);

	power_mode = BNO055_POWER_MODE_NORMAL;
	/* set the power mode as NORMAL*/
	comres += bno055_set_power_mode(power_mode);
	comres += bno055_get_power_mode(&power_mode_u8);
	
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
	osDelay(50);
  /* Infinite loop */
  for(;;)
  {	
		comres += bno055_get_operation_mode(&operation_mode_u8);
		if(operation_mode_u8 != BNO055_OPERATION_MODE_NDOF_FMC_OFF)
		{
			comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
			//comres += BNO055_ERROR;
			osDelay(8);
		}
		else
		{
			comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
//			comres += bno055_get_mag_calib_stat(&mag_calib_u8);
//			comres += bno055_get_gyro_calib_stat(&gyro_calib_u8);
//			comres += bno055_get_accel_calib_stat(&accel_calib_u8);
//			comres += bno055_get_sys_calib_stat(&system_calib_u8);
			
			yaw_ideiglenes = (float)d_euler_data_h;
			
			if(avarage_ind<50)
			{
				sum_yaw+=yaw_ideiglenes;
				avarage_ind++;
				if(avarage_ind==50)
				{
					sum_yaw=sum_yaw/50;
				}
			}
			else
			{
				yaw_ideiglenes -= sum_yaw - PI;
				if(yaw_ideiglenes<0)
				{
					yaw_ideiglenes += 2*PI;
				}
				else if(yaw_ideiglenes>2*PI)
				{
					yaw_ideiglenes -= 2*PI;
				}
				yaw = yaw_ideiglenes;
			}
			
			
		}
		
		

    osDelay(10);
  }
}

int8_t I2C_routine(void)
{
	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	return BNO055_INIT_VALUE;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	osDelay(msek);
}

/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  HAL_StatusTypeDef status = HAL_OK;
	s32 iError = BNO055_INIT_VALUE;

	//while (HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t)(dev_addr<<1), 3, I2CTIMEOUT) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&hi2c1,						// i2c handle
    						  (uint8_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(reg_data),		// write returned data to reg_data
							  cnt,							// write how many bytes
							  I2CTIMEOUT);							// timeout

	if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	return (s8)iError;
}

 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  HAL_StatusTypeDef status = HAL_OK;
	s32 iError = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
	u8 stringpos = BNO055_INIT_VALUE;
	array[BNO055_INIT_VALUE] = reg_addr;

	//while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dev_addr<<1), 3, I2CTIMEOUT) != HAL_OK) {}

    status = HAL_I2C_Mem_Read(&hi2c1,						// i2c handle
    						  (uint8_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&array),			// write returned data to this variable
							  cnt,							// how many bytes to expect returned
							  I2CTIMEOUT);							// timeout

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}

	return (s8)iError;
}

