#include "pid_controller.h"
#include "robot_geometry.h"
#include <math.h>
#include <stdbool.h>

// Reset PID Controller
void PIDReset(PIDController *pid) {
    // Reset integral terms
    pid->integralX = 0.0f;
    pid->integralY = 0.0f;
    
    // Reset derivative terms
    pid->derivativeX = 0.0f;
    pid->derivativeY = 0.0f;
    pid->prev_D_x = 0.0f;
    pid->prev_D_y = 0.0f;
    
    // Reset previous values
    pid->prev_err_x = 0.0f;
    pid->prev_err_y = 0.0f;
    pid->prev_out_x = 0.0f;
    pid->prev_out_y = 0.0f;
    pid->prev_time = HAL_GetTick();
    
    // Reset output values
    pid->output_theta = 0.0f;
    pid->output_phi = 0.0f;
    
    // Note: is_first_call reset is now handled in main loop
}

// Dat diem dich
void PIDSetTarget(PIDController *pid, float targetX, float targetY) {
    pid->setpointX = targetX;
    pid->setpointY = targetY;
}


// === NEW FUNCTION: PID WITH INTERNAL DT CALCULATION ===
// This function calculates dt internally using HAL_GetTick()
// Usage: tilt = PIDUpdateSimpleWithTime(&pid, currentX, currentY);
// No need to calculate dt in main loop anymore
float PIDUpdateSimpleWithTime(PIDController *pid, float currentX, float currentY) {
    // === 1. CALCULATE DT INTERNALLY ===
    uint32_t current_time = HAL_GetTick();
    float dt;
    
    // Handle first call (like Python: dt ≈ 0.033 if last_time is None)
    if (pid->prev_time == 0) {
        dt = 0.033f;  // Default dt for first call (30fps)
    } else {
        dt = (float)(current_time - pid->prev_time) / 1000.0f;  // Convert to seconds
    }

    // Clamp dt to a stable 30Hz window
    // - Too slow (>40ms) or invalid (<=0): force 33ms
    // - Too fast (<31.25ms): also force 33ms to avoid jitter amplification
    if (dt <= 0.0f || dt > 0.04f) {
        dt = 0.033f;   // Fixed 30Hz
    } else if (dt < 0.03125f) {
        dt = 0.033f;   // Ignore too-fast updates
    }
    
    // === 2. ERROR CALCULATION ===
    float err_x = -(currentX - pid->setpointX);
    float err_y = -(currentY - pid->setpointY);
    
    // === 2.5. DEADZONE ===
    /*float error_magnitude = sqrtf(err_x*err_x + err_y*err_y);
    if (error_magnitude < 3.0f) {  // Deadzone 3mm
        err_x = 0.0f;
        err_y = 0.0f;
    }*/
    
    // === 3. PROPORTIONAL TERM ===
    float P_x = pid->Kp * err_x;
    float P_y = pid->Kp * err_y;
    
    // === 4. DERIVATIVE TERM ===
    float d_err_x = 0.0f;
    float d_err_y = 0.0f;
    
    if (dt > 0.0f) {
        d_err_x = (err_x - pid->prev_err_x) / dt;
        d_err_y = (err_y - pid->prev_err_y) / dt;
    }
    
    // Direct derivative without filtering
    pid->derivativeX = d_err_x;
    pid->derivativeY = d_err_y;
    
    // Apply low-pass filter to derivative term
    float D_raw_x = pid->Kd * pid->derivativeX;
    float D_raw_y = pid->Kd * pid->derivativeY;
    float D_x = pid->d_alpha * D_raw_x + (1.0f - pid->d_alpha) * pid->prev_D_x;
    float D_y = pid->d_alpha * D_raw_y + (1.0f - pid->d_alpha) * pid->prev_D_y;
    
    // Store filtered derivatives for next iteration
    pid->prev_D_x = D_x;
    pid->prev_D_y = D_y;
    
    // === 5. INTEGRAL TERM ===
    float I_x = pid->Ki * pid->integralX;
    float I_y = pid->Ki * pid->integralY;
    
    // === 6. PID OUTPUT ===
    float pid_output_x = P_x + I_x + D_x;
    float pid_output_y = P_y + I_y + D_y;
    
    // === 7. OUTPUT FILTERING ===
    float filtered_x = pid->alpha * pid_output_x + (1.0f - pid->alpha) * pid->prev_out_x;
    float filtered_y = pid->alpha * pid_output_y + (1.0f - pid->alpha) * pid->prev_out_y;
    
    // === 8. CONVERT TO POLAR COORDINATES ===
    float phi = atan2f(filtered_y, filtered_x) * 180.0f / M_PI;
    if (phi < 0.0f) phi += 360.0f;
    float r = sqrtf(filtered_x * filtered_x + filtered_y * filtered_y);
    
    // === 9. MAGNITUDE CONVERSION ===
    float theta_unsaturated;
    float theta_saturated;
    
    // Calculate unsaturated theta based on conversion mode
    if (pid->magnitudeConvert == 1) {
        // Linear mode (giống Python)
        theta_unsaturated = pid->beta * r;
        // Giới hạn như Python: min(max(0, beta*r), max_theta)
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > pid->max_theta) {
            theta_saturated = pid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else if (pid->magnitudeConvert == 0) {
        // Tanh mode (giống Python) - dùng hệ số 2.4 trước tanh
        theta_unsaturated = 2.5f * tanhf(pid->beta * r);
        // Giới hạn như Python: max(0, 2.4*tanh(beta*r))
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else {
        // Default to linear
        theta_unsaturated = pid->beta * r;
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > pid->max_theta) {
            theta_saturated = pid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    }
    
    // === 10. SIMPLE INTEGRAL ACCUMULATION ===
    // Basic integral accumulation without anti-windup
    pid->integralX += err_x * dt;
    pid->integralY += err_y * dt;
    
    // === 11. SAVE VALUES FOR NEXT ITERATION ===
    pid->prev_err_x = err_x;
    pid->prev_err_y = err_y;
    pid->prev_out_x = filtered_x;
    pid->prev_out_y = filtered_y;
    pid->prev_time = current_time;  // Update time for next iteration
    
    // === 12. OUTPUT ===
    pid->output_theta = theta_saturated;
    pid->output_phi = phi;
    
    return theta_saturated;
}

// Initialize PID with new parameters
void PIDInitPythonStyle(PIDController *pid, float Kp, float Ki, float Kd, float alpha, float d_alpha, float beta, int conversion_mode) {
    // Basic PID gains
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // Initialize setpoints
    pid->setpointX = 0.0f;
    pid->setpointY = 0.0f;
    
    // Initialize integral terms
    pid->integralX = 0.0f;
    pid->integralY = 0.0f;
    
    // Initialize derivative terms
    pid->derivativeX = 0.0f;
    pid->derivativeY = 0.0f;
    pid->d_alpha = d_alpha;
    pid->prev_D_x = 0.0f;
    pid->prev_D_y = 0.0f;
    
    // Initialize previous values
    pid->prev_err_x = 0.0f;
    pid->prev_err_y = 0.0f;
    pid->prev_time = 0;  // Initialize to 0 for first call detection
    
    // Initialize output filter - truyền trực tiếp như Python
    pid->alpha = alpha;
    pid->prev_out_x = 0.0f;
    pid->prev_out_y = 0.0f;
    
    // Initialize magnitude conversion - truyền trực tiếp như Python
    pid->beta = beta;
    pid->max_theta = 15.0f;  // Default maximum theta value
    pid->magnitudeConvert = conversion_mode;  // 1=linear, 0=tanh
    
    // Initialize output values
    pid->output_theta = 0.0f;
    pid->output_phi = 0.0f;
}




// Set output filter coefficient
void PIDSetOutputFilter(PIDController *pid, float alpha) {
    pid->alpha = alpha;
}

// Set magnitude conversion parameters
void PIDSetMagnitudeParams(PIDController *pid, float beta, int conversion_mode) {
    pid->beta = beta;
    pid->magnitudeConvert = conversion_mode;  // 1=linear, 2=tanh
}




// === DERIVATIVE FILTER TUNING FUNCTION ===
// Set derivative filter coefficient
void PIDSetDerivativeFilter(PIDController *pid, float d_alpha) {
    pid->d_alpha = d_alpha;
}
