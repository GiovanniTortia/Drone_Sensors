#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sensor_lib/sensors.h"
#include <math.h>
#include <chrono>
#include <Eigen/Dense>


Eigen::Vector2d x;
Eigen::Vector3d u;
Eigen::Vector3d y;
double alpha = 0.5;
double Ts = 50e-3;
/*
Eigen::Matrix2d P;


Eigen::Matrix2d V1{
	{1e-2, 0},
	{0, 1e-2},
};
Eigen::Matrix3d V2{
	{0.004, 0, 0},
	{0, 0.0029, 0},
	{0, 0, 0.0021},
};

// Bruttino ma stica
Eigen::Matrix2d I_2{
	{1, 0},
	{0, 1},
};

/*void Kalman_RP(Eigen::Vector2d x_old){
	// Linearized matrices
	Eigen::Matrix2d A_bar{
		{u(1)*cos(x(0))*tan(x(1)) - u(2)*sin(x(0))*tan(x(1)), (u(1)*sin(x(0)) + u(2)*cos(x(0)))/(cos(x(1))*cos(x(1)))},
		{-u(1)*sin(x(0)), -u(2)*cos(x(1))},
	};
	Eigen::Matrix<double, 2,3> B_bar{
		{1, sin(x(0))*tan(x(1)), cos(x(0))*tan(x(1))},
		{0, cos(x(0)), -sin(x(1))},
	};
	Eigen::Matrix<double, 3,2> C_bar{
		{0, cos(x(1))},
		{cos(x(1))*cos(x(0)), -sin(x(1))*sin(x(0))},
		{cos(x(1))*sin(x(0)), sin(x(1))*cos(x(0))},
	};
	C_bar *= 9.80665;

	// Predictor-Corrector Kalman Filter
	Eigen::Matrix3d temp = C_bar*P*C_bar.transpose()+V2;
	Eigen::Matrix<double, 2,3> K0 = P*C_bar.transpose()*temp.inverse();

	Eigen::Matrix2d P0 = (I_2 - K0*C_bar)*P;
	Eigen::Vector3d e = y-C_bar*x_old;
	x = x_old + K0*e;
	printf("phi: %2.d, theta: %2.d\n", x(0), x(1));
	P = A_bar*P0*A_bar.transpose() + V1;
	x += 1e-2*(A_bar*x + B_bar*u);
}*/
void comp_filter(Eigen::Vector3d y_acc){
	Eigen::Matrix<double, 2,3> T{
		{1, sin(x(0))*tan(x(1)), cos(x(0))*tan(x(1))},
		{0, cos(x(0)), -sin(x(1))},
	};
	auto x_gyro = x + Ts*T*u;

	double phi_acc = atan2(y_acc(1), y_acc(2));
	double theta_acc = atan2(y_acc(0), y_acc(1)/sin(phi_acc));
	Eigen::Vector2d x_accel = {phi_acc, theta_acc};

	x = alpha*x_gyro + (1-alpha)*x_accel;
}

int main(){
	stdio_init_all();
	
	x.setZero();
	//P.setZero();
	
	i2c_init(i2c0, 400*1000);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
    
    sleep_ms(3000);
    printf("\n---------SONO SVEGLIO---------\n");
    sleep_ms(2000);	
	
	MPU6050 gyracc = MPU6050(i2c0);
	while(!gyracc.config()){
		printf("\nCouldn't properly configure the MPU6050, check connections\n");
		printf("Retrying in 10 seconds");
		for(int i=5;i>0;i--){
			sleep_ms(2000);
			printf(".");
		}
		printf("\n");
	}
	
	sleep_ms(1000);
	
	printf("Starting calibration...");
	
	gyracc.calibrate();
	
	printf("MPU6050 initialized!\n");
	
	int count = 0;
	double time = 0;
	auto begin = std::chrono::steady_clock::now();

	for(int i=0;i<1000;i++){
		gyracc.update();

		u(0) = gyracc.gyro[0];
		u(1) = gyracc.gyro[1];
		u(2) = gyracc.gyro[2];
		y(0) = gyracc.accel[0];
		y(1) = gyracc.accel[1];
		y(2) = gyracc.accel[2];

		comp_filter(y);
		printf("P: %2.f°, R: %2.f°\n", x(1), x(2));
		
		sleep_ms(45);
	}
	
	auto end = std::chrono::steady_clock::now();
	printf("Time difference = %d[µs]\n", std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
	return 0;
}
