# Data Protocol for C Code Communication

## Overview
This document describes the data frame structure for communication between C code (STM32) and Python GUI via UART2.

## Frame Structure
All data frames follow this format:
```
[START_BIT][DATA_TYPE][PARAMETER_DATA][STOP_BIT][CHECKSUM_LOW][CHECKSUM_HIGH]
```

### Frame Components:
- **START_BIT**: 0x40 ('@' character)
- **DATA_TYPE**: 1 byte identifier for data type
- **PARAMETER_DATA**: Variable length data based on data type
- **STOP_BIT**: 0x58 ('X' character)
- **CHECKSUM**: 2 bytes (little-endian), sum of all bytes from START_BIT to STOP_BIT

## Communication Directions

### Python → C (UART1)
- **Purpose**: Send ball coordinates from Python vision system to C control system
- **Format**: Simple 2-byte coordinates (X, Y)
- **Frequency**: Real-time (up to 50Hz)

### C → Python (UART2) 
- **Purpose**: Send control feedback (angles, PWM) from C system to Python GUI
- **Format**: Variable length data: 4 bytes (angles only) or 10 bytes (angles + PWM)
- **Frequency**: Real-time (up to 20Hz)

## Data Types

### 1. Ball Coordinates (Python → C)
**Frame Format:**
```
[0x40][X_INT8][Y_INT8][0x58][CHECKSUM_LOW][CHECKSUM_HIGH]
```

**Parameters:**
- X_INT8: X coordinate * 10 (signed 8-bit, range: -95 to +95 for ±9.5cm)
- Y_INT8: Y coordinate * 10 (signed 8-bit, range: -95 to +95 for ±9.5cm)

### 2. Angles and PWM (C → Python)
**Frame Format (with PWM for 3 servos - 10 bytes data):**
```
[0x40][THETA_INT16_LOW][THETA_INT16_HIGH][PHI_INT16_LOW][PHI_INT16_HIGH][PWM_S1_LOW][PWM_S1_HIGH][PWM_S2_LOW][PWM_S2_HIGH][PWM_S3_LOW][PWM_S3_HIGH][0x58][CHECKSUM_LOW][CHECKSUM_HIGH]
```

**Frame Format (angles only - 4 bytes data):**
```
[0x40][THETA_INT16_LOW][THETA_INT16_HIGH][PHI_INT16_LOW][PHI_INT16_HIGH][0x58][CHECKSUM_LOW][CHECKSUM_HIGH]
```

**Parameters:**
- THETA_INT16: Theta angle * 100 (signed 16-bit, range: -3000 to +3000 for ±30°)
- PHI_INT16: Phi angle * 100 (signed 16-bit, range: -3000 to +3000 for ±30°)
- PWM_S1: PWM value for Servo 1 (unsigned 16-bit, range: 0 to 65535)
- PWM_S2: PWM value for Servo 2 (unsigned 16-bit, range: 0 to 65535)
- PWM_S3: PWM value for Servo 3 (unsigned 16-bit, range: 0 to 65535)

## C Code Example

### Receiving Ball Coordinates from Python:
```c
typedef struct {
    uint8_t start_bit;      // 0x40
    int8_t x_int;          // X coordinate * 10
    int8_t y_int;          // Y coordinate * 10
    uint8_t stop_bit;      // 0x58
    uint16_t checksum;
} __attribute__((packed)) BallCoordinatesFrame;

bool receive_ball_coordinates(float* x, float* y) {
    BallCoordinatesFrame frame;
    
    // Receive frame via UART1
    if (HAL_UART_Receive(&huart1, (uint8_t*)&frame, sizeof(frame), 10) == HAL_OK) {
        
        // Verify frame
        if (frame.start_bit != 0x40 || frame.stop_bit != 0x58) {
            return false;
        }
        
        // Calculate and verify checksum
        uint16_t checksum = 0x40 + frame.x_int + frame.y_int + 0x58;
        if (frame.checksum != checksum) {
            return false;
        }
        
        // Convert to float coordinates
        *x = frame.x_int / 10.0f;
        *y = frame.y_int / 10.0f;
        
        return true;
    }
    
    return false;
}
```

### Sending Angles and PWM Data for 3 Servos:
```c
#include <stdint.h>

#define START_BIT 0x40
#define STOP_BIT 0x58

typedef struct {
    uint8_t start_bit;
    int16_t theta_int;
    int16_t phi_int;
    uint16_t pwm_s1;
    uint16_t pwm_s2;
    uint16_t pwm_s3;
    uint8_t stop_bit;
    uint16_t checksum;
} __attribute__((packed)) AnglesPWMFrame;

void send_angles_pwm_3servos(float theta, float phi, uint16_t pwm_s1, uint16_t pwm_s2, uint16_t pwm_s3) {
    AnglesPWMFrame frame;
    
    // Fill frame data
    frame.start_bit = START_BIT;
    frame.theta_int = (int16_t)(theta * 100);  // Convert to int16
    frame.phi_int = (int16_t)(phi * 100);      // Convert to int16
    frame.pwm_s1 = pwm_s1;
    frame.pwm_s2 = pwm_s2;
    frame.pwm_s3 = pwm_s3;
    frame.stop_bit = STOP_BIT;
    
    // Calculate checksum
    uint16_t checksum = 0;
    uint8_t* data_ptr = (uint8_t*)&frame;
    for (int i = 0; i < 13; i++) {  // From start_bit to stop_bit (13 bytes)
        checksum += data_ptr[i];
    }
    frame.checksum = checksum;
    
    // Send frame via UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)&frame, sizeof(frame), 100);
}

// Example usage:
void main_loop() {
    float theta = -1.23;  // degrees
    float phi = 2.45;     // degrees
    uint16_t pwm_s1 = 1500;  // Servo 1 PWM
    uint16_t pwm_s2 = 1600;  // Servo 2 PWM
    uint16_t pwm_s3 = 1400;  // Servo 3 PWM
    
    send_angles_pwm_3servos(theta, phi, pwm_s1, pwm_s2, pwm_s3);
    
    HAL_Delay(50);  // Send every 50ms
}
```

### Complete Example - Main Loop:
```c
void main_loop() {
    float ball_x, ball_y;
    float theta = 0.0f, phi = 0.0f;
    uint16_t pwm_s1 = 1500, pwm_s2 = 1500, pwm_s3 = 1500;
    
    while (1) {
        // 1. Receive ball coordinates from Python (UART1)
        if (receive_ball_coordinates(&ball_x, &ball_y)) {
            printf("Received ball: X=%.1f, Y=%.1f\n", ball_x, ball_y);
            
            // 2. Calculate control angles and PWM values
            theta = calculate_theta(ball_x, ball_y);
            phi = calculate_phi(ball_x, ball_y);
            pwm_s1 = calculate_servo1_pwm(theta, phi);
            pwm_s2 = calculate_servo2_pwm(theta, phi);
            pwm_s3 = calculate_servo3_pwm(theta, phi);
            
            // 3. Send angles and PWM back to Python (UART2)
            send_angles_pwm_3servos(theta, phi, pwm_s1, pwm_s2, pwm_s3);
        }
        
        HAL_Delay(20);  // 50Hz control loop
    }
}
```

### Sending Angles Only:
```c
typedef struct {
    uint8_t start_bit;
    int16_t theta_int;
    int16_t phi_int;
    uint8_t stop_bit;
    uint16_t checksum;
} __attribute__((packed)) AnglesOnlyFrame;

void send_angles_only(float theta, float phi) {
    AnglesOnlyFrame frame;
    
    frame.start_bit = START_BIT;
    frame.theta_int = (int16_t)(theta * 100);
    frame.phi_int = (int16_t)(phi * 100);
    frame.stop_bit = STOP_BIT;
    
    // Calculate checksum
    uint16_t checksum = 0;
    uint8_t* data_ptr = (uint8_t*)&frame;
    for (int i = 0; i < 6; i++) {  // From start_bit to stop_bit (6 bytes)
        checksum += data_ptr[i];
    }
    frame.checksum = checksum;
    
    // Send frame via UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)&frame, sizeof(frame), 100);
}
```

## Python GUI Display

The Python GUI will automatically:
1. Parse incoming frames from UART2
2. Decode θ, φ, and 3 servo PWM values
3. Display them in the Hercules terminal window with color coding:
   - **Yellow**: Angles and PWM data
   - **Orange**: PWM-specific data
   - **Red**: Error messages
4. Update real-time value displays showing:
   - θ: Theta angle in degrees
   - φ: Phi angle in degrees
   - S1: Servo 1 PWM value
   - S2: Servo 2 PWM value
   - S3: Servo 3 PWM value

## Example Terminal Output
```
[12:34:56.789] Angles: θ=-1.23°, φ=2.45° | PWM: S1=1500, S2=1600, S3=1400
[12:34:56.839] Angles: θ=-1.45°, φ=2.67° | PWM: S1=1520, S2=1580, S3=1420
```

## Notes

- All multi-byte values are in little-endian format
- Checksum is calculated as the sum of all bytes from START_BIT to STOP_BIT
- **Ball coordinates (Python→C)**: 2-byte format (X, Y coordinates)
- **Angles/PWM (C→Python)**: Variable length format detected automatically by Python
  - 4 bytes: angles only (θ, φ)
  - 10 bytes: angles + PWM (θ, φ, S1, S2, S3)
- No data type identifiers needed - frame type detected by length
- Communication is optimized for real-time display with minimal CPU impact
- Two separate UART channels: UART1 for coordinates, UART2 for feedback 