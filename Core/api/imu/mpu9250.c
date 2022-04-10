/*
 * mpu9250.c
 *
 *  Created on: 12 Mar 2022
 *      Author: onias
 */
#include "MPU9250.h"



const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint8_t _address = 0b11010000;
// 400 kHz
const uint32_t _i2cRate = 400000;

static uint8_t _buffer[21];
static uint8_t _mag_adjust[3];

imu_status_e imu_mpu9250_ctor(imu_mpu9250_t* const mpu9250, SPI_HandleTypeDef *hspi, io_pin_t ncs, io_pin_t irq)
{
	//TODO: write vtable
	static const struct imu_vtable vtable = {
			(imu_status_e (*)(	imu_t * const mpu9250))&imu_mpu9250_getDataVTable,
			(imu_status_e (*)(	imu_t * const mpu9250))&imu_mpu9250_readAccelVTable,
			(imu_status_e (*)(	imu_t * const mpu9250))&imu_mpu9250_readGyroVTable,
			(imu_status_e (*)(	imu_t * const mpu9250))&imu_mpu9250_readTempVTable,
		};
	imu_status_e status = IMU_ctor(&mpu9250->imu);
	mpu9250->imu.vptr = &vtable;
	mpu9250->imu.sensor = IMU_MPU_9250;
	mpu9250->imu.status = IMU_NOT_INIT;
	mpu9250->hspi = hspi;
	mpu9250->ncs = ncs;
	mpu9250->irq = irq;
	return status;
}


static inline void MPU9250_Activate(imu_mpu9250_t* const mpu9250)
{
	HAL_GPIO_WritePin(mpu9250->ncs.gpio_port, mpu9250->ncs.gpio_pin, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate(imu_mpu9250_t* const mpu9250)
{
	HAL_GPIO_WritePin(mpu9250->ncs.gpio_port, mpu9250->ncs.gpio_pin, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(imu_mpu9250_t* const mpu9250, uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(mpu9250->hspi,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MPU_SPI_Write(imu_mpu9250_t* const mpu9250, uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate(mpu9250);
	SPIx_WriteRead(mpu9250, WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(mpu9250,*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate(mpu9250);
}

void MPU_SPI_Read(imu_mpu9250_t* const mpu9250, uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate(mpu9250);
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(mpu9250->hspi, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(mpu9250->hspi, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate(mpu9250);
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(imu_mpu9250_t* const mpu9250, uint8_t subAddress, uint8_t data)
{
	MPU_SPI_Write(mpu9250, &data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(imu_mpu9250_t* const mpu9250, uint8_t subAddress, uint8_t count, uint8_t* dest){
	MPU_SPI_Read(mpu9250, dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(imu_mpu9250_t* const mpu9250, uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(mpu9250, I2C_SLV0_ADDR,AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(mpu9250, I2C_SLV0_REG,subAddress);

	// store the data for write
	writeRegister(mpu9250, I2C_SLV0_DO,data);

	// enable I2C and send 1 byte
	writeRegister(mpu9250, I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(imu_mpu9250_t* const mpu9250, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(mpu9250, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(mpu9250, I2C_SLV0_REG,subAddress);

	// enable I2C and request the bytes
	writeRegister(mpu9250, I2C_SLV0_CTRL,I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(mpu9250, EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(imu_mpu9250_t* const mpu9250){
	// read the WHO AM I register
	readRegisters(mpu9250, WHO_AM_I,1,_buffer);

	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(imu_mpu9250_t* const mpu9250){
	// read the WHO AM I register
	readAK8963Registers(mpu9250, AK8963_WHO_AM_I,1,_buffer);
	// return the register value
	return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t imu_mpu9250_Init(imu_mpu9250_t* const mpu9250)
{
	// select clock source to gyro
	writeRegister(mpu9250, PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(mpu9250, USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(mpu9250, I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(mpu9250, PWR_MGMNT_1,PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(mpu9250, AK8963_CNTL2,AK8963_RESET);
	// select clock source to gyro
	writeRegister(mpu9250, PWR_MGMNT_1,CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI(mpu9250);
	if((who != 0x71) &&( who != 0x73))
	{
		return 1;
	}

	// enable accelerometer and gyro
	writeRegister(mpu9250, PWR_MGMNT_2,SEN_ENABLE);

	// setting accel range to 16G as default
	writeRegister(mpu9250, ACCEL_CONFIG,ACCEL_FS_SEL_16G);

	// setting the gyro range to 2000DPS as default
	writeRegister(mpu9250, GYRO_CONFIG,GYRO_FS_SEL_250DPS);

	// setting bandwidth to 184Hz as default
	writeRegister(mpu9250, ACCEL_CONFIG2,DLPF_184);

	// setting gyro bandwidth to 184Hz
	writeRegister(mpu9250, CONFIG,DLPF_184);

	// setting the sample rate divider to 0 as default
	writeRegister(mpu9250, SMPDIV,0x00);

	// enable I2C master mode
	writeRegister(mpu9250, USER_CTRL,I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(mpu9250, I2C_MST_CTRL,I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963(mpu9250) != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(mpu9250, AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(mpu9250, PWR_MGMNT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(mpu9250, AK8963_HXL,7,_buffer);

	// successful init, return 0
	return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(imu_mpu9250_t* const mpu9250, AccelRange range)
{
	writeRegister(mpu9250, ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(imu_mpu9250_t* const mpu9250, GyroRange range)
{
	writeRegister(mpu9250, GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(imu_mpu9250_t* const mpu9250, DLPFBandwidth bandwidth)
{
	writeRegister(mpu9250, ACCEL_CONFIG2,bandwidth);
	writeRegister(mpu9250, CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(imu_mpu9250_t* const mpu9250, SampleRateDivider srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(mpu9250, SMPDIV,19);

	if(srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(mpu9250, AK8963_HXL,7,_buffer);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(mpu9250, AK8963_CNTL1,AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(mpu9250, AK8963_HXL,7,_buffer);
	}

	writeRegister(mpu9250, SMPDIV, srd);
}

/* read the data, each argiment should point to a array for x, y, and x */
imu_status_e imu_mpu9250_getDataVTable(imu_mpu9250_t* const mpu9250)
{
	// grab the data from the MPU9250
	readRegisters(mpu9250, ACCEL_OUT, 21, _buffer);

	// combine into 16 bit values
	mpu9250->imu.Accel_X_RAW = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	mpu9250->imu.Accel_Y_RAW = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	mpu9250->imu.Accel_Z_RAW = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	mpu9250->imu.Gyro_X_RAW = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	mpu9250->imu.Gyro_Y_RAW = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	mpu9250->imu.Gyro_Z_RAW = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	int16_t magx = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	int16_t magy = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	int16_t magz = (((int16_t)_buffer[19]) << 8) | _buffer[18];

	mpu9250->Mag_X_RAW = (int16_t)((float)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
	mpu9250->Mag_Y_RAW = (int16_t)((float)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
	mpu9250->Mag_Z_RAW = (int16_t)((float)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));

	return mpu9250->imu.status;
}

imu_status_e imu_mpu9250_readAccelVTable(	imu_mpu9250_t * const mpu9250)
{
	return mpu9250->imu.status;
}
imu_status_e imu_mpu9250_readGyroVTable(	imu_mpu9250_t * const mpu9250)
{
	return mpu9250->imu.status;
}
imu_status_e imu_mpu9250_readTempVTable(	imu_mpu9250_t * const mpu9250)
{
	return mpu9250->imu.status;
}
