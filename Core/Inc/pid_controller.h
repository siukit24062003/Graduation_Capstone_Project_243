#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "robot_geometry.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Cau truc PID Controller
typedef struct {
    // Basic PID parameters
    float Kp, Ki, Kd;
    float setpointX, setpointY;
    
    // Integral terms
    float integralX, integralY;  // Integral terms
    
    // Derivative terms
    float derivativeX, derivativeY;  // Previous derivative values
    
    // Derivative low-pass filter
    float d_alpha;                   // Derivative filter coefficient (0..1)
    float prev_D_x, prev_D_y;       // Previous filtered derivative values
    
    // Previous values for calculations
    float prev_err_x, prev_err_y;
    float prev_time;
    // Note: is_first_call removed - now handled in main loop
    
    // Output filtering (keep from original)
    float alpha;                // Output filter coefficient
    float prev_out_x, prev_out_y;
    
    // Magnitude conversion
    float beta;                 // Conversion coefficient
    float max_theta;           // Maximum tilt angle
    int magnitudeConvert;      // 1=linear, 2=tanh
    
    // Output values (for simplified PID)
    float output_theta;        // Calculated tilt angle
    float output_phi;          // Calculated phi angle
} PIDController;

// Nguyen mau ham - Only active functions
void PIDReset(PIDController *pid);
void PIDSetTarget(PIDController *pid, float targetX, float targetY);

// Python-style PID functions (currently used)
void PIDUpdatePythonStyle(PIDController *pid, float currentX, float currentY, float dt, float *outputTilt, float *outputPhi);
void PIDInitPythonStyle(PIDController *pid, float Kp, float Ki, float Kd, float alpha, float d_alpha, float beta, int conversion_mode);


void PIDSetOutputFilter(PIDController *pid, float alpha);
void PIDSetMagnitudeParams(PIDController *pid, float beta, int conversion_mode);

// Simplified PID Controller (Python Style)
float PIDUpdateSimple(PIDController *pid, float currentX, float currentY, float dt);

// New function that calculates dt internally
float PIDUpdateSimpleWithTime(PIDController *pid, float currentX, float currentY);




// === DERIVATIVE FILTER TUNING FUNCTIONS ===
void PIDSetDerivativeFilter(PIDController *pid, float d_alpha);  // Set derivative filter coefficient

#endif /* PID_CONTROLLER_H */
