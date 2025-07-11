#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"


//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				MPU6050
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------


#define MPU_SELF 0x68

//ACCELEROMETERS OUTPUT REGISTERS, BOTH THESE AND THE GYRO'S ARE COMPOSED OF 2 8b REGISTERS
#define ACCEL_XOUT_H 	0x3B
#define ACCEL_YOUT_H 	0x3D
#define ACCEL_ZOUT_H 	0x3F

//GYRO OUTPUT REGISTERS
#define GYRO_XOUT_H		0x43
#define GYRO_YOUT_H		0x45
#define GYRO_ZOUT_H 		0x47

//CONFIGURATION REGISTERS
#define MPU_SMPLRT_DIV 	0x19
#define MPU_CONFIG 	0x1A
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG 	0x1C
#define MPU_PWR_MGMT_1 	0x6B


//INTERRUPT REGISTERS
#define MPU_INT_STATUS 	0X3A
#define MPU_INT_ENABLE 	0X38

//resolutions
#define ACC_RES 		16384.0f
#define GYRO_RES 		131.0f

//misc
#define MPU_WHO_AM_I 	0x75

class MPU6050{
	public:
		MPU6050(i2c_inst_t *i2c);
		void update();
		bool config();
		void calibrate();
		
		bool isReady();
		
		double accel[3];
		double gyro[3];
		
	private:
		i2c_inst_t *i2c;
		double g_offset[3];
		double angle[3];
		
		
		size_t read_reg(uint8_t addr, uint8_t *dst, size_t len);
		size_t write_reg(uint8_t addr, uint8_t src);
};







//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				BMP280
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------


#define BMP_SELF 				0x76			//only if SD0 is connected to ground
#define REG_PRESSURE_MSB 		0xF7
#define BMP_CONFIG 				0xF5
#define CTRL_MEAS 				0xF4
#define BMP_STATUS 				0xF3
#define BMP_RESET 				0xE0
#define BMP_ID 					0xD0
#define REG_DIG_T1_LSB 			0x88


#define BMP2_MIN_TEMP_DOUBLE	-40.0f
#define BMP2_MAX_TEMP_DOUBLE	85.0f
#define BMP2_MIN_PRES_DOUBLE	30000.0f
#define BMP2_MAX_PRES_DOUBLE	110000.0f



class BMP280{
	public:
		BMP280(i2c_inst_t *i2c);
		bool config();													//correctly sets all configuration registers
		void update();													//updates the measurements stored in the object
		double get_temp();												
		double get_pressure();
		double get_sea_alt();											//returns the altitude compared to the sea level
		double get_rel_alt();											//returns the altitude compared to the floor
	
	private:
		i2c_inst_t *i2c;
		double temp;
		double pressure;
		double floor;
		
		size_t read_reg(uint8_t addr, uint8_t *dst, size_t len);
		size_t write_reg(uint8_t addr, uint8_t src);
		
		void compensate_meas(int32_t raw_press, int32_t raw_temp);		//compensates the measurements according to calibration parameters
		void get_calib_params();										//reads the calibration parameters from their registers
		bool meas_ready();												//returns true if a new measurement is read
		
		
		
		// temperature params
    	uint16_t dig_t1;
    	int16_t dig_t2;
    	int16_t dig_t3;

		// pressure params
		uint16_t dig_p1;
		int16_t dig_p2;
		int16_t dig_p3;
		int16_t dig_p4;
		int16_t dig_p5;
		int16_t dig_p6;
		int16_t dig_p7;
		int16_t dig_p8;
		int16_t dig_p9;		
};


//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				QMC5883L
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------


#define QMC_SELF			0x0D
#define QMC_XOUT_LSB		0x00
#define QMC_STATUS_1		0x06
#define QMC_TEMP_OUT_LSB	0x07
#define QMC_CONTROL_1		0x09
#define QMC_CONTROL_2		0x0A
#define QMC_SR_PERIOD		0x0B
#define QMC_STATUS_2		0x0C
#define QMC_CHIP_ID			0x0D			//must read 0xFF
#define QMC_RESOLUTION		2.0/32767.0f	//only for full scale of 2G

class QMC5883L{
	public:
		QMC5883L(i2c_inst_t *i2c);
		bool config();
		double get_hdn();
		void update();
		double *get_field();
		void calibrate();
		
	private:
		double mag_field[3];
		i2c_inst_t *i2c;
		bool meas_ready();
		size_t read_reg(uint8_t addr, uint8_t *dst, size_t len);
		size_t write_reg(uint8_t addr, uint8_t src);
};
