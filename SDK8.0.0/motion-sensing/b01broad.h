/* Copyright (c) 2015 MtM Technology Corporation. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#ifndef B01BROAD_H
#define B01BROAD_H

// Pressure Sensor
#define PRES_SCL         2
#define PRES_SDA         3
#define PRES_ADDR        0xEE

// Temperature & Humidity 
#define THUM_SCL         2
#define THUM_SDA         3
#define THUM_ADDR        0x80

// 3-axis Gyroscope
#define GYRO_SCL         13
#define GYRO_SDA         14
#define GYRO_INT2         9
#define GYRO_ADDR        0xD0

// 3-axis Accerlerometer
#define ACC_SCL          13
#define ACC_SDA          14
#define ACC_INT2         8
#define ACC_ADDR         0x30

//LED Pin Define
#define LED_GREEN        15
#define LED_RED          16
#define LED_BLUE         6


#define ADC_1           4     
#define ADC_2           5
#define PWM2            0
#define PWM1            1


#endif
