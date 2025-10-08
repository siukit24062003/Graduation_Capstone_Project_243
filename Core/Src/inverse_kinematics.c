#include "inverse_kinematics.h"
#include <stdbool.h>

// Tinh diem A tren mat phang tren cua robot
void SolveTop(RobotState *robot, float a, float b, float c, float h) {
    float denom1 = sqrtf(4.0f * SQUARE(c) + SQUARE(a - SQRT3 * b));
    robot->E.x = -(robot->geom.lp * c) / denom1;
    robot->E.y = (SQRT3 * robot->geom.lp * c) / denom1;
    robot->E.z = h + ((a - SQRT3 * b) * robot->geom.lp) / denom1;

    float denom2 = sqrtf(SQUARE(c) + SQUARE(a));
    robot->D.x = (robot->geom.lp * c) / denom2;  
    robot->D.y =  0.0f;
    robot->D.z = h - ((robot->geom.lp * a) / denom2);

    float denom3 = sqrtf(4.0f * SQUARE(c) + SQUARE(a + SQRT3 * b));
    robot->F.x = -(robot->geom.lp * c) / denom3;
    robot->F.y = -(SQRT3 * robot->geom.lp * c) / denom3;
    robot->F.z = h + ((a + SQRT3 * b) * robot->geom.lp) / denom3;
}

void SolveMiddle(RobotState *robot) {
    // Calculations for Middle Point Q (corresponds to Top Point E)
    float Q1 = (-robot->E.x + SQRT3 * robot->E.y - 2.0f * robot->geom.lb) / robot->E.z;
    float Q2 = (SQUARE(robot->E.x) + SQUARE(robot->E.y) + SQUARE(robot->E.z) + SQUARE(robot->geom.l2) -
                SQUARE(robot->geom.l1) - SQUARE(robot->geom.lb)) / (2.0f * robot->E.z);
    float Q3 = SQUARE(Q1) + 4.0f;
    float Q4 = 2.0f * Q1 * Q2 + 4.0f * robot->geom.lb;
    float Q5 = SQUARE(Q2) + SQUARE(robot->geom.lb) - SQUARE(robot->geom.l2);
    float Q_disc = SQUARE(Q4) - 4.0f * Q3 * Q5;
    if (Q_disc < 0.0f) Q_disc = 0.0f;

    robot->Q.x = (-Q4 - sqrtf(Q_disc)) / (2.0f * Q3);
    robot->Q.y = -(SQRT3) * robot->Q.x;
    robot->Q.z = sqrtf(SQUARE(robot->geom.l2) - 4.0f * SQUARE(robot->Q.x) - 4.0f * robot->geom.lb * robot->Q.x - SQUARE(robot->geom.lb));

    // Calculations for Middle Point P (corresponds to Top Point D)
    float P1 = (robot->geom.lb - robot->D.x) / robot->D.z;
    float P2 = (SQUARE(robot->D.x) + SQUARE(robot->D.z) - SQUARE(robot->geom.lb) +
                SQUARE(robot->geom.l2) - SQUARE(robot->geom.l1)) / (2.0f * robot->D.z);
    float P3 = SQUARE(P1) + 1.0f;
    float P4 = 2.0f * (P1 * P2 - robot->geom.lb);
    float P5 = SQUARE(P2) - SQUARE(robot->geom.l2) + SQUARE(robot->geom.lb);
    float P_disc = SQUARE(P4) - 4.0f * P3 * P5;
    if (P_disc < 0.0f) P_disc = 0.0f;

    robot->P.x = (-P4 + sqrtf(P_disc)) / (2.0f * P3);
    robot->P.y = 0.0f;
    robot->P.z = sqrtf(SQUARE(robot->geom.l2) - SQUARE(robot->P.x - robot->geom.lb));

    // Calculations for Middle Point R (corresponds to Top Point F)
    float R1 = (-robot->F.x - SQRT3 * robot->F.y - 2.0f * robot->geom.lb) / robot->F.z;
    float R2 = (SQUARE(robot->F.x) + SQUARE(robot->F.y) + SQUARE(robot->F.z) + SQUARE(robot->geom.l2) -
                SQUARE(robot->geom.l1) - SQUARE(robot->geom.lb)) / (2.0f * robot->F.z);
    float R3 = SQUARE(R1) + 4.0f;
    float R4 = 2.0f * R1 * R2 + 4.0f * robot->geom.lb;
    float R5 = SQUARE(R2) + SQUARE(robot->geom.lb) - SQUARE(robot->geom.l2);
    float R_disc = SQUARE(R4) - 4.0f * R3 * R5;
    if (R_disc < 0.0f) R_disc = 0.0f;

    robot->R.x = (-R4 - sqrtf(R_disc)) / (2.0f * R3);
    robot->R.y = (SQRT3) * robot->R.x;
    robot->R.z = sqrtf(SQUARE(robot->geom.l2) - 4.0f * SQUARE(robot->R.x) - 4.0f * robot->geom.lb * robot->R.x - SQUARE(robot->geom.lb));
}

void SolveInverseKinematicsVector(RobotState *robot, float a, float b, float c, float h) {

    robot->B_base.x = -0.5f * robot->geom.lb;            // B₁: -Lb/2
    robot->B_base.y = SQRT3 * 0.5f * robot->geom.lb;     // B₁: √3·Lb/2
    robot->B_base.z = 0.0f;

    robot->A_base.x = robot->geom.lb;                     // B₂: Lb (SWAPPED
    robot->A_base.y = 0.0f;                               // B₂: 0 (SWAPPED
    robot->A_base.z = 0.0f;

    robot->C_base.x = -0.5f * robot->geom.lb;            // B₃: -Lb/2
    robot->C_base.y = -SQRT3 * 0.5f * robot->geom.lb;    // B₃: -√3·Lb/2
    robot->C_base.z = 0.0f;

    SolveTop(robot, a, b, c, h);
    SolveMiddle(robot);

    //robot->theta[0] = PI/2.0f - atan2f(sqrtf(SQUARE(robot->Q.x) + SQUARE(robot->Q.y)) - robot->geom.lb, robot->Q.z);
    //robot->theta[1] = atan2f(robot->P.z, robot->P.x - robot->geom.lb);
    //robot->theta[2] = PI/2.0f - atan2f(sqrtf(SQUARE(robot->R.x) + SQUARE(robot->R.y)) - robot->geom.lb, robot->R.z);
    robot->theta[0] = atan2f(robot->Q.z, sqrtf(SQUARE(robot->Q.x) + SQUARE(robot->Q.y)) - robot->geom.lb);
    robot->theta[1] = atan2f(robot->P.z, robot->P.x - robot->geom.lb);
    robot->theta[2] = atan2f(robot->R.z, sqrtf(SQUARE(robot->R.x) + SQUARE(robot->R.y)) - robot->geom.lb);
}

// Giai dong hoc nguoc tu goc nghieng (theta, phi)
void SolveInverseKinematicsSpherical(RobotState *robot, float theta_deg, float phi_deg, float h) {

    robot->geom.h = h;
    ComputeMaxThetaAtHeight(robot, h);
    
    theta_deg = MIN(theta_deg, robot->geom.maxtheta);
    
    // MATCH MIKE.PY: Special case for theta near 0

    float theta = DEG_TO_RAD(theta_deg);
    float phi = DEG_TO_RAD(phi_deg);

    float a = sinf(theta) * cosf(phi);
    float b = sinf(theta) * sinf(phi);
    float c = cosf(theta);

    SolveInverseKinematicsVector(robot, a, b, c, h);
}

void ComputeMaxThetaAtHeight(RobotState *robot, float h) {
    float theta_low = 0.0f;
    float theta_high = DEG_TO_RAD(20.0f);
    float tol = 1e-3f;

    // Safety check for height
    if (h <= 0.0f || h > 15.0f) {
        robot->geom.maxtheta = 12.0f;  // Fallback value
        return;
    }

    while (theta_high - theta_low > tol) {
        float theta_mid = (theta_low + theta_high) / 2.0f;
        bool valid = true;
        
        // Test both directions like Python: for s in (1, -1)
        for (int s_sign = 1; s_sign >= -1 && valid; s_sign -= 2) {
        float c = cosf(theta_mid);
            float s = (float)s_sign * sinf(theta_mid);

        // Test coordinates for top point D
        float D_x_test = robot->geom.lp * c;
        float D_z_test = h - robot->geom.lp * s;
            
            if (D_z_test <= 0.0f) {
                valid = false;
                break;
            }

        // Test calculations for middle point P
        float P1_test = (robot->geom.lb - D_x_test) / D_z_test;
        float P2_test = (SQUARE(D_x_test) + SQUARE(D_z_test) - SQUARE(robot->geom.lb) +
                    SQUARE(robot->geom.l2) - SQUARE(robot->geom.l1)) / (2.0f * D_z_test);
        float P3_test = SQUARE(P1_test) + 1.0f;
        float P4_test = 2.0f * (P1_test * P2_test - robot->geom.lb);
        float P5_test = SQUARE(P2_test) - SQUARE(robot->geom.l2) + SQUARE(robot->geom.lb);
        float P_disc_test = SQUARE(P4_test) - 4.0f * P3_test * P5_test;

        if (P_disc_test < 0.0f) {
                valid = false;
                break;
        }

        float P_x_test = (-P4_test + sqrtf(P_disc_test)) / (2.0f * P3_test);
        float delta_P = SQUARE(robot->geom.l2) - SQUARE(P_x_test - robot->geom.lb);

        if (delta_P < 0.0f) {
                valid = false;
                break;
        }

        float P_z_test = sqrtf(delta_P);
        float d1 = sqrtf(SQUARE(D_x_test - P_x_test) + SQUARE(D_z_test - P_z_test));
        float d2 = sqrtf(SQUARE(robot->geom.lb - P_x_test) + SQUARE(P_z_test));

            if (fabsf(d1 - robot->geom.l1) > 1e-3f || fabsf(d2 - robot->geom.l2) > 1e-3f) {
                valid = false;
                break;
            }
        }
        
        if (valid) {
            theta_low = theta_mid;
        } else {
            theta_high = theta_mid;
        }
    }

    robot->geom.maxtheta = MAX(0.0f, RAD_TO_DEG(theta_low) - 0.5f);  // Match Python safety margin
}
