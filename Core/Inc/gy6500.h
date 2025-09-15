/*
 * gy6500.h
 *
 *  Created on: Sep 14, 2025
 *      Author: ugie01
 */

#ifndef INC_GY6500_H_
#define INC_GY6500_H_

#include "main.h"
#include "i2c.h"
#include <stdio.h>
#include <stdbool.h>

#define DEV_ADDR 		(0x68 << 1)

// REG
#define WHO_AM_I 		0x75
#define ACCEL_XOUT_H  	0x3B
#define GYRO_XOUT_H   	0x43
#define TEMP_OUT_H    	0x41
#define PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG	0x1C
#define CONFIG     	 	0x1A
#define GYRO_CONFIG	   	0x1B
#define ACCEL_CONFIG	0x1C
#define XA_OFFSET_H    0x77
#define XA_OFFSET_L    0x78
#define YA_OFFSET_H    0x7A
#define YA_OFFSET_L    0x7B
#define ZA_OFFSET_H    0x7D
#define ZA_OFFSET_L    0x7E
#define XG_OFFSET_H    0x13
#define XG_OFFSET_L    0x14
#define YG_OFFSET_H    0x15
#define YG_OFFSET_L    0x16
#define ZG_OFFSET_H    0x17
#define ZG_OFFSET_L    0x18

// ON && OFF
#define ACCEL_X_DIS (1 << 5) // 0b00100000
#define ACCEL_Y_DIS (1 << 4) // 0b00010000
#define ACCEL_Z_DIS (1 << 3) // 0b00001000
#define GYRO_X_DIS  (1 << 2) // 0b00000100
#define GYRO_Y_DIS  (1 << 1) // 0b00000010
#define GYRO_Z_DIS  (1 << 0) // 0b00000001
#define TEMP_ON     true     // 0b00001000
#define TEMP_DIS    false    // 0b00001000

// ALL ON && OFF
#define ALL_SENSORS_ON  (0x00)
#define ALL_GYRO_DIS    (GYRO_X_DIS | GYRO_Y_DIS | GYRO_Z_DIS)
#define ALL_ACCEL_DIS   (ACCEL_X_DIS | ACCEL_Y_DIS | ACCEL_Z_DIS)

typedef struct _GyData
{
    // 사용할 I2C 포트
    I2C_HandleTypeDef *hi2c;

    // 사용할 슬레이브 주소
    uint8_t address;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp;

    // 계산된 오프셋 값 저장용 변수
    int16_t accel_offset_x, accel_offset_y, accel_offset_z;
    int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;

    // 계산된 각도 (Roll, Pitch, Yaw)
    float roll, pitch, yaw;

    // 6bit - temp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z - 0bit	 ---> 	  0 활성화 , 1 비활성화
    uint8_t state;
} GyData;

// 자이로스코프 측정 범위
typedef enum
{
    GYRO_RANGE_250_DPS = 0, // +-250 도/s
    GYRO_RANGE_500_DPS = 1, // +-500 도/s
    GYRO_RANGE_1000_DPS = 2, // +-1000 도/s
    GYRO_RANGE_2000_DPS = 3  // +-2000 도/s
} gyro_range_t;

// 가속도계 측정 범위
typedef enum
{
    ACCEL_RANGE_2G = 0, // +-2g
    ACCEL_RANGE_4G = 1, // +-4g
    ACCEL_RANGE_8G = 2, // +-8g
    ACCEL_RANGE_16G = 3  // +-16g
} accel_range_t;

// 디지털 저역 통과 필터
typedef enum
{
    DLPF_BW_260_HZ = 0, // 260 Hz
    DLPF_BW_184_HZ = 1, // 184 Hz
    DLPF_BW_94_HZ = 2, // 94 Hz
    DLPF_BW_44_HZ = 3, // 44 Hz
    DLPF_BW_21_HZ = 4, // 21 Hz
    DLPF_BW_10_HZ = 5, // 10 Hz
    DLPF_BW_5_HZ = 6  // 5 Hz
} dlpf_mode_t;

void Scanning_I2C();
HAL_StatusTypeDef Gy6500_init(GyData *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address);
HAL_StatusTypeDef getAllData(GyData *dev);
HAL_StatusTypeDef getAccelerometer(GyData *dev);
HAL_StatusTypeDef getGyroscope(GyData *dev);
HAL_StatusTypeDef getTemp(GyData *dev);
HAL_StatusTypeDef setSensorState(GyData *dev, uint8_t setData);
HAL_StatusTypeDef setTempState(GyData *dev, bool set_data);
HAL_StatusTypeDef setGyroRange(GyData *dev, gyro_range_t range);
HAL_StatusTypeDef setAccelRange(GyData *dev, accel_range_t range);
HAL_StatusTypeDef setDLPF(GyData *dev, dlpf_mode_t mode);
HAL_StatusTypeDef calibrate(GyData *dev, uint16_t num_samples);
HAL_StatusTypeDef setGyroOffsets(GyData *dev);
HAL_StatusTypeDef setAccelOffsets(GyData *dev);
void updateAngles(GyData *dev, float dt);

#endif /* INC_GY6500_H_ */
