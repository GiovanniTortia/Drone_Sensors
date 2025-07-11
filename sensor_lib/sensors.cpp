#include "sensors.h"

//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				MPU6050
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------


MPU6050::MPU6050(i2c_inst_t *i2c){
	this -> i2c = i2c;
	for(int i=0;i<3;i++){
		this -> g_offset[i] = 0;
		this -> angle[i] = 0;
	}
}

//reads len bytes from register at address addr and stores them in dst
size_t MPU6050::read_reg(uint8_t addr, uint8_t *dst, size_t len){
	i2c_write_blocking(this -> i2c, MPU_SELF, &addr, 1, true);
	return i2c_read_blocking(this -> i2c, MPU_SELF, dst, len, false);;
}

size_t MPU6050::write_reg(uint8_t addr, uint8_t src){
	uint8_t buf[2] = {addr, src};
	return i2c_write_blocking(this -> i2c, MPU_SELF, buf, 2, false);
}

//configures operating mode, osr, accelerometer and gyroscope (check MPU6050 doc)
bool MPU6050::config(){
	
	//MPU6050::write_reg(MPU_PWR_MGMT_1, 0x80);	//reset

	//test connection
	uint8_t test;
	MPU6050::read_reg(MPU_WHO_AM_I, &test, 1);
	if(test != 0x68){
		printf("Valore inaspettato di WHO_AM_I: %d\n", test);
		return false;
	}
		
	//MPU6050::write_reg(MPU_PWR_MGMT_1, 0x80);	//reset
	//sleep_ms(5000);
		
	printf("smplrt: %d\n", MPU6050::write_reg(MPU_SMPLRT_DIV, 0x00));			//no samplerate divider
	printf("config: %d\n", MPU6050::write_reg(MPU_CONFIG, 0x02));				//no fsync, low low pass filter action
	printf("gyro config: %d\n", MPU6050::write_reg(GYRO_CONFIG, 0x00));			//250°/s full scale	=> 131 LSB/(°/s)
	printf("accel config: %d\n", MPU6050::write_reg(ACCEL_CONFIG, 0x00));		//2g full scale => 16384 LSB/g
	printf("int enable: %d\n", MPU6050::write_reg(MPU_INT_ENABLE, 0x01));		//enable data ready interrupt
	printf("pwr mgmt: %d\n", MPU6050::write_reg(MPU_PWR_MGMT_1, 0x09));			//pll as recommended in docs, disable sleep mode, temperature disabled

	MPU6050::read_reg(MPU_PWR_MGMT_1, &test, 1);
	if(test != 0x09){
		printf("\nMPU_PWR_MGMT_1: %u", test);
		return false;
	}
		
	MPU6050::read_reg(MPU_INT_ENABLE, &test, 1);
	if(test != 0x01){
		printf("\nMPU_INT_ENABLE: %u", test);
		return false;
	}
		
	MPU6050::read_reg(ACCEL_CONFIG, &test, 1);
	if(test != 0x00){
		printf("\nACCEL_CONFIG: %u", test);
		return false;
	}
		
	MPU6050::read_reg(GYRO_CONFIG, &test, 1);
	if(test != 0x00){
		printf("\nGYRO_CONFIG: %u", test);
		return false;
	}
	
		
	MPU6050::read_reg(MPU_CONFIG, &test, 1);
	if(test != 0x02){
		printf("\nMPU_CONFIG: %u", test);
		return false;
	}
		
	MPU6050::read_reg(MPU_SMPLRT_DIV, &test, 1);
	if(test != 0x00){
		printf("\nMPU_SMPLRT_DIV: %u", test);
		return false;
	}
	
	return true;
}

void MPU6050::update(){
	while(!MPU6050::isReady());
	uint8_t raw[14];
	MPU6050::read_reg(ACCEL_XOUT_H, raw, 14);
	
	this -> accel[0] = (int16_t)((raw[0] << 8) | raw[1]) / ACC_RES;
	this -> accel[1] = (int16_t)((raw[2] << 8) | raw[3]) / ACC_RES;
	this -> accel[2] = (int16_t)((raw[4] << 8) | raw[5]) / ACC_RES;
	
	this -> gyro[0] = (int16_t)((raw[8] << 8) | raw[9]) / GYRO_RES - this->g_offset[0];
	this -> gyro[1] = (int16_t)((raw[10] << 8) | raw[11]) / GYRO_RES - this->g_offset[1];
	this -> gyro[2] = (int16_t)((raw[12] << 8) | raw[13]) / GYRO_RES - this->g_offset[2];
	
}

//calculates the offsets for the gyro and the initial angles based on the accelerometer
void MPU6050::calibrate(){
	printf("\nStarting calibration, keep the sensor still\n");
	sleep_ms(500);
	double offset[3] = {0, 0, 0};
	for(int count=0;count<5000;count++){
		while(!MPU6050::isReady());
		MPU6050::update();
		offset[0] += this -> gyro[0];
		offset[1] += this -> gyro[1];
		offset[2] += this -> gyro[2];
	}
	
	this -> g_offset[0] = offset[0]/5000;
	this -> g_offset[1] = offset[1]/5000;
	this -> g_offset[2] = offset[2]/5000;
}

//returns true whether fresh data can be read
bool MPU6050::isReady(){
	uint8_t drdy;
	MPU6050::read_reg(MPU_INT_STATUS, &drdy, 1);
	return (drdy & 1) == 1;
}

/*double *MPU6050::getGyro(float *read){

}

double *MPU6050::getAccel(){
	return null;
}*/

//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				BMP280
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------

BMP280::BMP280(i2c_inst_t *i2c){
	this -> i2c = i2c;
}

size_t BMP280::read_reg(uint8_t addr, uint8_t *dst, size_t len){
	i2c_write_blocking(this -> i2c, BMP_SELF, &addr, 1, true);
	return i2c_read_blocking(this -> i2c, BMP_SELF, dst, len, false);
}

size_t BMP280::write_reg(uint8_t addr, uint8_t src){
	uint8_t buf[2] = {addr, src};
	return i2c_write_blocking(this -> i2c, BMP_SELF, buf, 2, false);
}

double BMP280::get_temp(){
	return temp;
}

double BMP280::get_pressure(){
	return pressure;
}

double BMP280::get_sea_alt(){
	return 44330 * (1 - pow(this->pressure/101325.0, 1/5.255));
}

double BMP280::get_rel_alt(){
	return get_sea_alt() - floor;
}

bool BMP280::config(){
	write_reg(BMP_RESET, 0xB6);		//reset
	sleep_ms(100);

	uint8_t test;
	read_reg(BMP_ID, &test, 1);		//test connection	
	if(test != 0x58)
		return false;
									
	write_reg(BMP_CONFIG, 0x00);	//4x iir filter, 0.5ms between reads, no spi
	write_reg(CTRL_MEAS, 0x7F);		//temp osr x4, press osr x16, normal mode
	get_calib_params();
	
	read_reg(BMP_CONFIG, &test, 1);
	if(test != 0x00)
		return false;
		
	read_reg(CTRL_MEAS, &test, 1);
	if(test != 0x7F)
		return false;
	
	floor = 0;						//calculate floor level for reference
	for(int i=0;i<5000;i++){
		//while(!meas_ready()){printf("no cisn\n");}
		update();
		floor += pressure;
	}
	floor /= 5000;
	floor = 44330 * (1 - pow(floor/101325.0, 1/5.255));
	
	return true;
}

bool BMP280::meas_ready(){
	uint8_t status;
	read_reg(BMP_STATUS, &status, 1);
	return ((status >> 3) & 1) == 0;
}

void BMP280::update(){
	uint8_t buf[6];
	//while(!meas_ready());		//0.5ms probably too short to check
    read_reg(REG_PRESSURE_MSB, buf, 6);

    // store the 20 bit read in a 32 bit signed integer for conversion
    int32_t raw_pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t raw_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
	compensate_meas(raw_pressure, raw_temp);
}

void BMP280::compensate_meas(int32_t raw_pressure, int32_t raw_temperature){

    double var1, var2;

	//temperature compensation
    var1 = (((double) raw_temperature) / 16384.0 - ((double) dig_t1) / 1024.0) *
           ((double) dig_t2);
    var2 =
        ((((double) raw_temperature) / 131072.0 - ((double) dig_t1) / 8192.0) *
         (((double) raw_temperature) / 131072.0 - ((double) dig_t1) / 8192.0)) *
        ((double) dig_t3);

    int32_t t_fine = (int32_t) (var1 + var2);
    
    this->temp = (var1 + var2) / 5120.0;

    if (this->temp < BMP2_MIN_TEMP_DOUBLE)
        this->temp = BMP2_MIN_TEMP_DOUBLE;

    if (this->temp > BMP2_MAX_TEMP_DOUBLE)
        this->temp = BMP2_MAX_TEMP_DOUBLE;
   	
   	//pressure compensation
    var1 = ((double) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double) dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double) dig_p4) * 65536.0);
    var1 = (((double)dig_p3) * var1 * var1 / 524288.0 + ((double)dig_p2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) dig_p1);

    if (var1 < 0 || var1 > 0)
    {
    	this->pressure = 0.0;
        this->pressure = 1048576.0 - (double)raw_pressure;
        this->pressure = (this->pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)dig_p9) * this->pressure * this->pressure / 2147483648.0;
        var2 = this->pressure * ((double)dig_p8) / 32768.0;

        this->pressure = this->pressure + (var1 + var2 + ((double)dig_p7)) / 16.0;

        if (this->pressure < BMP2_MIN_PRES_DOUBLE)
            this->pressure = BMP2_MIN_PRES_DOUBLE;

        if (this->pressure > BMP2_MAX_PRES_DOUBLE)
            this->pressure = BMP2_MAX_PRES_DOUBLE;
    }
}

void BMP280::get_calib_params() {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB and MSB register

    uint8_t buf[24] = {0};
    read_reg(REG_DIG_T1_LSB, buf, 24);
	
	sleep_ms(5000);

    // store these in a struct for later use
    dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}

//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                                           				QMC5883L
//***************************************************************************************************************************************
//---------------------------------------------------------------------------------------------------------------------------------------

QMC5883L::QMC5883L(i2c_inst_t *i2c){
	this -> i2c = i2c;
}

size_t QMC5883L::read_reg(uint8_t addr, uint8_t *dst, size_t len){
	i2c_write_blocking(this -> i2c, QMC_SELF, &addr, 1, true);
	return i2c_read_blocking(this -> i2c, QMC_SELF, dst, len, false);
}

size_t QMC5883L::write_reg(uint8_t addr, uint8_t src){
	uint8_t buf[2] = {addr, src};
	return i2c_write_blocking(this -> i2c, QMC_SELF, buf, 2, false);
}

double *QMC5883L::get_field(){
	return mag_field;
}

bool QMC5883L::config(){
	//test connection
	uint8_t test;		
	read_reg(QMC_CHIP_ID, &test, 1);
	if(test != 0xFF)
		return false;
		
	write_reg(QMC_CONTROL_2, 0x80);		//soft reset	
	write_reg(QMC_CONTROL_1, 0X0D);		//OSR: 512x, continuous mode, 2G full scale, 200Hz ODR
	write_reg(QMC_CONTROL_2, 0x00);		//pointer roll-over function disabled, interrupt pin disabled	
	write_reg(QMC_SR_PERIOD, 0x01);		//recommended in the datasheet for continuous mode
	
	read_reg(QMC_CONTROL_1, &test, 1);
	if(test != 0x0D)
		return false;
		
	read_reg(QMC_CONTROL_2, &test, 1);
	if(test != 0x00)
		return false;
		
	read_reg(QMC_SR_PERIOD, &test, 1);
	if(test != 0x01)
		return false;
		
	return true;
}

double QMC5883L::get_hdn(){
	double hdn = atan2(mag_field[1], mag_field[0]) * 180 / M_PI;
	if(hdn < 0)
		hdn += 360;
	return hdn;
}

bool QMC5883L::meas_ready(){
	uint8_t test;
	read_reg(QMC_STATUS_1, &test, 1);
	return (test & 1) == 1;
}

void QMC5883L::update(){
	while(!meas_ready());	//wait for measurement
	uint8_t raw[6];
	read_reg(QMC_XOUT_LSB, raw, 6);
	for(int i=0;i<3;i++){
		mag_field[i] = (int16_t)((raw[2*i+1] << 8)|raw[2*i]) * QMC_RESOLUTION;
	}
}


