// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Mouse.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double total_angle_x=0;
double total_angle_y=0;
int16_t x_position = 0;
int16_t y_position = 0;
int16_t counter = 0;
#define LED_PIN 13


#define AX_ZERO (300) //Zero offset for ax
#define GX_ZERO (-59) //Zero offset for gy
#define AY_ZERO (-580)//Zero offset for ay
#define GY_ZERO (1560)//Zero offset for gx


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    //Mouse.begin();
}

//Final result from the Kalman_Filter
float Angle_x=0.0;
float Angle_y=0.0;
//Result from gyroscope angle
float Angle_gy=0.0;
float Angle_gx=0.0;

//Kalman Filter Stuff For X direction
float Q_angle_x=0.9;
float Q_gyro_x=0.001;
float R_angle_x=0.5;
float dt=0.01;
char  C_0_x = 1;
float Q_bias_x, Angle_err_x;
float PCt_0_x=0.0, PCt_1_x=0.0, E_x=0.0;
float K_0_x=0.0, K_1_x=0.0, t_0_x=0.0, t_1_x=0.0;
float Pdot_x[4] ={0,0,0,0};
float PP_x[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_x(float Accel,float Gyro)
{
    Angle_x+=(Gyro - Q_bias_x) * dt;

    Pdot_x[0]=Q_angle_x - PP_x[0][1] - PP_x[1][0];

    Pdot_x[1]=- PP_x[1][1];
    Pdot_x[2]=- PP_x[1][1];
    Pdot_x[3]=Q_gyro_x;

    PP_x[0][0] += Pdot_x[0] * dt;
    PP_x[0][1] += Pdot_x[1] * dt;
    PP_x[1][0] += Pdot_x[2] * dt;
    PP_x[1][1] += Pdot_x[3] * dt;

    Angle_err_x = Accel - Angle_x;

    PCt_0_x = C_0_x * PP_x[0][0];
    PCt_1_x = C_0_x * PP_x[1][0];

    E_x = R_angle_x + C_0_x * PCt_0_x;

    if(E_x!=0)
    {
      K_0_x = PCt_0_x / E_x;
      K_1_x = PCt_1_x / E_x;
    }

    t_0_x = PCt_0_x;
    t_1_x = C_0_x * PP_x[0][1];

    PP_x[0][0] -= K_0_x * t_0_x;
    PP_x[0][1] -= K_0_x * t_1_x;
    PP_x[1][0] -= K_1_x * t_0_x;
    PP_x[1][1] -= K_1_x * t_1_x;

    Angle_x += K_0_x * Angle_err_x;
    Q_bias_x += K_1_x * Angle_err_x;
}

//Kalman Filter Stuff For X direction
float Q_angle_y=0.9;
float Q_gyro_y=0.001;
float R_angle_y=0.5;
char  C_0_y = 1;
float Q_bias_y, Angle_err_y;
float PCt_0_y=0.0, PCt_1_y=0.0, E_y=0.0;
float K_0_y=0.0, K_1_y=0.0, t_0_y=0.0, t_1_y=0.0;
float Pdot_y[4] ={0,0,0,0};
float PP_y[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_y(float Accel,float Gyro)
{
    Angle_y+=(Gyro - Q_bias_y) * dt;

    Pdot_y[0]=Q_angle_y - PP_y[0][1] - PP_y[1][0];

    Pdot_y[1]=- PP_y[1][1];
    Pdot_y[2]=- PP_y[1][1];
    Pdot_y[3]=Q_gyro_y;

    PP_y[0][0] += Pdot_y[0] * dt;
    PP_y[0][1] += Pdot_y[1] * dt;
    PP_y[1][0] += Pdot_y[2] * dt;
    PP_y[1][1] += Pdot_y[3] * dt;

    Angle_err_y = Accel - Angle_y;

    PCt_0_y = C_0_y * PP_y[0][0];
    PCt_1_y = C_0_y * PP_y[1][0];

    E_y = R_angle_y + C_0_y * PCt_0_y;

    if(E_y!=0)
    {
      K_0_y = PCt_0_y / E_y;
      K_1_y = PCt_1_y / E_y;
    }

    t_0_y = PCt_0_y;
    t_1_y = C_0_y * PP_y[0][1];

    PP_y[0][0] -= K_0_y * t_0_y;
    PP_y[0][1] -= K_0_y * t_1_y;
    PP_y[1][0] -= K_1_y * t_0_y;
    PP_y[1][1] -= K_1_y * t_1_y;

    Angle_y += K_0_y * Angle_err_y;
    Q_bias_y += K_1_y * Angle_err_y;
}



void loop() {
    // read raw accel/gyro measurements from device
    double a_angle_x=0.0;
    double g_angle_x=0.0;
    double a_angle_y=0.0;
    double g_angle_y=0.0;
    unsigned long time=0;
    unsigned long mictime=0;
    static unsigned long pretime=0;
    float gyro_x=0.0;
    float gyro_y=0.0;
    if(pretime==0)
    {
        pretime=millis();
        return;
    }
    mictime=millis();

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /* 加速度量程范围设置2g 16384 LSB/g
    * 计算公式：
    * 前边已经推导过这里再列出来一次
    * x是小车倾斜的角度,y是加速度计读出的值
    * sinx = 0.92*3.14*x/180 = y/16384
    * x=180*y/(0.92*3.14*16384)=
    */

    ax -= AX_ZERO;
    a_angle_x=ax/262;
    ay -= AY_ZERO;
    a_angle_y=ay/262;

    /* 陀螺仪量程范围设置250 131 LSB//s
    * 陀螺仪角度计算公式:
    * 小车倾斜角度是gx_angle,陀螺仪读数是y,时间是dt
    * gx_angle +=(y/(131*1000))*dt
    */
    gy -= GX_ZERO;
    gx -= GY_ZERO;
    time=mictime-pretime;

    gyro_x=gy/131.0;
    g_angle_x=gyro_x*time;
    g_angle_x=g_angle_x/1000.0;
    total_angle_x-=g_angle_x;

    gyro_y=gx/131.0;
    g_angle_y=gyro_y*time;
    g_angle_y=g_angle_y/1000.0;
    total_angle_y-=g_angle_y;

    dt=time/1000.0;
    Kalman_Filter_x(a_angle_x,gyro_x);
    Kalman_Filter_y(a_angle_y,gyro_y);

    Serial.print("x_angle:\t");
    Serial.print(a_angle_x); Serial.print(",");
    Serial.print(total_angle_x); Serial.print(",");
    Serial.println(Angle_x);

    Serial.print("y_angle:\t");
    Serial.print(a_angle_y); Serial.print(",");
    Serial.print(total_angle_y); Serial.print(",");
    Serial.println(Angle_y);

    Serial.print("time for one loop: ");
    Serial.println(time);

    // Move the position of the mouse. Counter is used to avoid moving the mouse too fast
    if (counter == 9){
       if(Angle_x < 7 && Angle_x > -7){
          x_position = 0;
       }
       else if (Angle_x < 20 && Angle_x > -20){
          x_position = Angle_x > 0 ? 10 : - 10;
       }
       else if (Angle_x < 40 && Angle_x > -40){
          x_position = Angle_x > 0 ?  30 :  - 30;
       }
       else{
          x_position = Angle_x > 0 ?  60 :  - 60;
       }

       if(Angle_y < 7 && Angle_y > -7){
          y_position = 0;
       }
       else if (Angle_y < 20 && Angle_y > -20){
         y_position = Angle_y > 0 ? 10 :  - 10;
       }
       else if (Angle_y < 40 && Angle_y > -40){
          y_position = Angle_y > 0 ? 30 :  - 30;
       }
       else{
          y_position = Angle_y > 0 ? 60 :  - 60;
       }

       //Mouse.move(x_position, y_position, 0);
       counter = 0;
    }
    else{
      counter ++;
    }

    Serial.print("x_position: ");
    Serial.println(x_position);

    Serial.print("y_position: ");
    Serial.println(y_position);

    pretime=mictime;
}
