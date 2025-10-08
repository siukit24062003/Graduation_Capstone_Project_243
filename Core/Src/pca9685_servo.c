#include "pca9685_servo.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern I2C_HandleTypeDef hi2c1;

// Global variables to track current servo angles (in degrees) and PWM values
float current_servo_angles[3] = {0.0f, 0.0f, 0.0f};
uint16_t current_servo_pwm[3] = {0, 0, 0};

// PCA9685 definitions from working code
#define PCA9685_ADDRESS 0x80
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

// ========= LOW-LEVEL ==========
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_StatusTypeDef status;

    // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
    status = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (status != HAL_OK) {
        // I2C Read failed
        return;
    }

    if (Value == 0) readValue &= ~(1 << Bit);
    else readValue |= (1 << Bit);

    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (status != HAL_OK) {
        // I2C Write failed
        return;
    }
    HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
    uint8_t prescale;
    if(frequency >= 1526) prescale = 0x03;
    else if(frequency <= 24) prescale = 0xFF;
    //  internal 25 MHz oscillator as in the datasheet page no 1/52
    else prescale = 25000000 / (4096 * frequency);
    // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 50); // Tăng timeout
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
    PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress;
    uint8_t pwm[4];
    HAL_StatusTypeDef status;

    registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
    // See example 1 in the datasheet page no 18/52
    // Mask to ensure 12-bit values only
    OnTime &= 0x0FFF;
    OffTime &= 0x0FFF;

    pwm[0] = OnTime & 0xFF;
    pwm[1] = OnTime >> 8;
    pwm[2] = OffTime & 0xFF;
    pwm[3] = OffTime >> 8;

    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 50); // Tăng timeout lên 50ms
    if (status != HAL_OK) {
        // I2C Write failed - PWM not set
        return;
    }
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
    float Value;
    // 50 Hz servo then 4095 Value --> 20 milliseconds
    // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
    Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
    PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

// ========= CALIBRATE + CONVERT ==========
static inline uint16_t us_to_tick(uint16_t us) {
    // Convert microseconds to PCA9685 ticks
    // For 50Hz: 20ms = 4096 ticks, so 1us = 4096/20000 ticks
    return (uint16_t)(us * 4096.0f / 20000.0f);
}

// ---- Servo 1 ----
uint16_t Servo1_AngleToTick(float angle_deg) {
    // Use lookup table from calibration document for 0-90 degree range (ascending angles)
     const float angle_table[] = {0 ,6, 10, 13, 16.5, 21, 25, 30, 33, 37, 40, 45, 50, 53.5, 59, 62.5, 65, 74, 79, 83, 87.5, 90};
   //const uint16_t pwm_table[] = {1900, 1860, 1810, 1760, 1710, 1660, 1610, 1560, 1510, 1460, 1410, 1360, 1310, 1260, 1210, 1160, 1110, 1060, 1010, 960, 910, 860, 800};
     const uint16_t pwm_table[] = {1900, 1850, 1800, 1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000, 950, 900, 850,800};
   //const uint16_t pwm_table[] = {1900, 1840, 1790, 1740, 1690, 1640, 1590, 1540, 1490, 1440, 1390, 1340, 1290, 1240, 1190, 1140, 1090, 1040, 990, 940, 890, 840, 800};
  
     int n = (int)(sizeof(angle_table)/sizeof(angle_table[0]));


    // Clamp to table bounds
    if (angle_deg <= angle_table[0]) return us_to_tick(pwm_table[0]);
    if (angle_deg >= angle_table[n-1]) return us_to_tick(pwm_table[n-1]);

    // Linear interpolation within the table (angles ascending)
    for (int i = 0; i < n-1; i++) {
        if (angle_deg >= angle_table[i] && angle_deg < angle_table[i+1]) {
            float t = (angle_deg - angle_table[i]) / (angle_table[i+1] - angle_table[i]);
            float pwm = pwm_table[i] + t * (pwm_table[i+1] - pwm_table[i]);
            return us_to_tick((uint16_t)pwm);
        }
    }
    // Fallback (should not reach)
    return us_to_tick(pwm_table[n-1]);
}

// ---- Servo 2 ----
uint16_t Servo2_AngleToTick(float angle_deg) {
	 const float angle_table[] = {0, 8, 11.5, 15.5, 18, 22.5, 26.5, 31, 35, 38.5, 42, 47, 51, 55, 60, 65, 69, 72.5, 76.5, 82, 85.5, 90};
//   const uint16_t pwm_table[] = {1770, 1730, 1680, 1630, 1580, 1530, 1480, 1430, 1380, 1330, 1280, 1230, 1180, 1130, 1080, 1030, 980, 930, 880, 830, 780, 720};
	 const uint16_t pwm_table[] = {1770, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000, 950, 900, 850, 800, 750, 720};
//   const uint16_t pwm_table[] = {1770, 1690, 1640, 1590, 1540, 1490, 1440, 1390, 1340, 1290, 1240, 1190, 1140, 1090, 1040, 990, 940, 890, 840, 790, 740, 720};

    int n = sizeof(angle_table)/sizeof(angle_table[0]);

    if (angle_deg <= 0) return us_to_tick(pwm_table[0]);
    if (angle_deg >= 90) return us_to_tick(pwm_table[n-1]);

    for (int i = 0; i < n-1; i++) {
        if (angle_deg >= angle_table[i] && angle_deg < angle_table[i+1]) {
            float t = (angle_deg - angle_table[i]) / (angle_table[i+1] - angle_table[i]);
            float pwm = pwm_table[i] + t * (pwm_table[i+1] - pwm_table[i]);
            return us_to_tick((uint16_t)pwm);
        }
    }
    return us_to_tick(pwm_table[n-1]);
}

// ---- Servo 3 ----
uint16_t Servo3_AngleToTick(float angle_deg) {
//    const float angle_table[] = {0, 4, 8, 11, 15, 19, 23, 28, 31, 36, 41, 45, 49, 54, 57, 64, 67, 72, 75, 82, 90};
	const float angle_table[] = {0, 5, 9, 13, 18, 22, 26, 29, 34, 38.5, 43, 46.5, 51, 55.5, 60, 64.5, 70, 73, 77.5, 82.5, 86.5, 90};
	const uint16_t pwm_table[] = {1860, 1800, 1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000, 950, 900, 850, 810};
//	const uint16_t pwm_table[] = {1860, 1790, 1740, 1690, 1640, 1590, 1540, 1490, 1440, 1390, 1340, 1290, 1240, 1190, 1140, 1090, 1040, 990, 940, 890, 840, 810};
//const uint16_t pwm_table[] = {1860, 1810, 1760, 1710, 1660, 1610, 1560, 1510, 1460, 1410, 1360, 1310, 1260, 1210, 1160, 1110, 1060, 1010, 960, 910, 860, 810};

	int n = sizeof(angle_table)/sizeof(angle_table[0]);

    if (angle_deg <= 0) return us_to_tick(pwm_table[0]);
    if (angle_deg >= 90) return us_to_tick(pwm_table[n-1]);

    for (int i = 0; i < n-1; i++) {
        if (angle_deg >= angle_table[i] && angle_deg < angle_table[i+1]) {
            float t = (angle_deg - angle_table[i]) / (angle_table[i+1] - angle_table[i]);
            float pwm = pwm_table[i] + t * (pwm_table[i+1] - pwm_table[i]);
            return us_to_tick((uint16_t)pwm);
        }
    }
    return us_to_tick(pwm_table[n-1]);
}

// ========= SERVO CONTROL =========
void SetServoAngles(float theta1, float theta2, float theta3) {
    float theta1_deg = theta1 * 180.0f / M_PI;
    float theta2_deg = theta2 * 180.0f / M_PI;
    float theta3_deg = theta3 * 180.0f / M_PI;

    // CRITICAL: Giới hạn góc servo trong khoảng 0-90 độ
    if (theta1_deg < 0.0f) theta1_deg = 0.0f;
    if (theta1_deg > 90.0f) theta1_deg = 90.0f;
    
    if (theta2_deg < 0.0f) theta2_deg = 0.0f;
    if (theta2_deg > 90.0f) theta2_deg = 90.0f;
    
    if (theta3_deg < 0.0f) theta3_deg = 0.0f;
    if (theta3_deg > 90.0f) theta3_deg = 90.0f;

    // Update current angles tracking
    current_servo_angles[0] = theta1_deg;
    current_servo_angles[1] = theta2_deg;
    current_servo_angles[2] = theta3_deg;

    uint16_t tick1 = Servo1_AngleToTick(theta1_deg);
    uint16_t tick2 = Servo2_AngleToTick(theta2_deg);
    uint16_t tick3 = Servo3_AngleToTick(theta3_deg);

    // Update current PWM values tracking
    current_servo_pwm[0] = tick1;
    current_servo_pwm[1] = tick2;
    current_servo_pwm[2] = tick3;

    // Send PWM to PCA9685
    PCA9685_SetPWM(SERVO1_CHANNEL, 0, tick1);  // Channel 0
    PCA9685_SetPWM(SERVO2_CHANNEL, 0, tick2);  // Channel 1
    PCA9685_SetPWM(SERVO3_CHANNEL, 0, tick3);  // Channel 2
}

// Test function for servo channels - moves through kinematic range
void TestServoChannels(void) {
    // Test each servo individually through their typical working range
    for (int servo = 0; servo < 3; servo++) {
        // Move to 30 degrees (typical working position)
        PCA9685_SetServoAngle(servo, 30.0f);
        HAL_Delay(1000);

        // Move to 0 degrees
        PCA9685_SetServoAngle(servo, 0.0f);
        HAL_Delay(1000);

        // Move to 60 degrees (upper working range)
        PCA9685_SetServoAngle(servo, 60.0f);
        HAL_Delay(1000);

        // Return to typical working position
        PCA9685_SetServoAngle(servo, 30.0f);
        HAL_Delay(500);
    }
}

// Simple function to set individual servo angle (for testing)
void SetIndividualServo(uint8_t servo_num, float angle_deg) {
    if (servo_num == 0) {
        current_servo_angles[0] = angle_deg;
        uint16_t tick = Servo1_AngleToTick(angle_deg);
        current_servo_pwm[0] = tick;
        PCA9685_SetPWM(SERVO1_CHANNEL, 0, tick);
    } else if (servo_num == 1) {
        current_servo_angles[1] = angle_deg;
        uint16_t tick = Servo2_AngleToTick(angle_deg);
        current_servo_pwm[1] = tick;
        PCA9685_SetPWM(SERVO2_CHANNEL, 0, tick);
    } else if (servo_num == 2) {
        current_servo_angles[2] = angle_deg;
        uint16_t tick = Servo3_AngleToTick(angle_deg);
        current_servo_pwm[2] = tick;
        PCA9685_SetPWM(SERVO3_CHANNEL, 0, tick);
    }
}

// Test I2C connectivity
bool PCA9685_TestConnection(void) {
    uint8_t testValue;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &testValue, 1, 100);
    return (status == HAL_OK);
}
