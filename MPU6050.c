// MPU6050 required Registers

#define W_DATA         0xD0
#define R_DATA         0xD1
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define SMPRT_DIV      0x19
#define CONFIG_R       0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define ACCEL_XOUT_L   0x3C
#define ACCEL_YOUT_H   0x3D
#define ACCEL_YOUT_L   0x3E
#define ACCEL_ZOUT_H   0x3F
#define ACCEL_ZOUT_L   0x40
#define TEMP_OUT_H     0x41
#define TEMP_OUT_L     0x42
#define GYRO_XOUT_H    0x43
#define GYRO_XOUT_L    0x44
#define GYRO_YOUT_H    0x45
#define GYRO_YOUT_L    0x46
#define GYRO_ZOUT_H    0x47
#define GYRO_ZOUT_L    0x48

void mpu6050_write(int add, int data)
{
         i2c_start();
         i2c_write(W_DATA);
         i2c_write(add);
         i2c_write(data);
         i2c_stop();
 
}
      
int16 mpu6050_read(int add){
         int retval;
         i2c_start();
         i2c_write(W_DATA);
         i2c_write(add);
         i2c_start();
         i2c_write(R_DATA);
         retval=i2c_read(0);
         i2c_stop();
         return retval;
}
 
void mpu6050_init(){
         mpu6050_write(PWR_MGMT_1,  0x80);
         delay_ms(100);
         mpu6050_write(PWR_MGMT_1,  0x00);
         delay_ms(100);
         mpu6050_write(CONFIG_R,    0x01);
         delay_ms(10);
         mpu6050_write(GYRO_CONFIG, 0x00);
}
