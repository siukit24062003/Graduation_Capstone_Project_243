/**
 * ball_and_plate.c
 * Implementation of main API for ball and plate system
 */

 #include "ball_and_plate.h"
 #include "robot_geometry.h"
 #include "inverse_kinematics.h"
 #include "servo_control.h"
 #include <math.h>
 
 // Initialize robot with default geometry parameters
 void RobotInit(RobotState *robot) {
     // Set geometry parameters for the robot (in cm) - Updated for new hardware
     robot->geom.lp = 9.4f;    // Top platform radius (updated from 6.0f)
     robot->geom.l1 = 9.0f;    // Upper arm length (updated from 12.0f) 
     robot->geom.l2 = 4.5f;    // Lower arm length (updated from 12.0f)
     robot->geom.lb = 5.5f;    // Base platform radius (updated from 8.0f)
     
     // Calculate height limits - Optimized for current hardware
     float theoretical_maxh = ComputeMaxH(&robot->geom);
     float theoretical_minh = ComputeMinH(&robot->geom);
     
     // Apply margins consistent with Python project
     robot->geom.maxh = theoretical_maxh - 0.2f;  // Match Python: maxh = compute_maxh() - 0.2
     robot->geom.minh = theoretical_minh + 0.45f; // Match Python: minh = compute_minh() + 0.45
     
     // Ensure valid height range with hardware-specific checks
     if (robot->geom.minh <= 0.0f || robot->geom.minh >= robot->geom.maxh) {
         // Hardware-specific fallback values (measured empirically)
         robot->geom.minh = 9.4f;   // Safe minimum for current hardware
         robot->geom.maxh = 12.7f;   // Safe maximum for current hardware
     }
     
     // Set working height like Python: midpoint between min and max
     robot->geom.h = 0.5f * (robot->geom.maxh + robot->geom.minh);
     
     // Set maximum tilt angle (degrees) - Match Python software limit
     robot->geom.maxtheta = 10.0f;
     
     // Initialize servo control
     ServoInit();
     
     // Compute maximum tilt angle at current height
     ComputeMaxThetaAtHeight(robot, robot->geom.h);
     
         // Initialize robot to level position (0 tilt, 0 phi) at working height
    // This calculates the theoretical servo angles for a level platform
    SolveInverseKinematicsSpherical(robot, 0.0f, 0.0f, robot->geom.h);
    
    // Set servos to their calculated theoretical positions immediately
    // θ1 ≈ 28.1°, θ2 ≈ 27.3°, θ3 ≈ 28.1° for level platform at h ≈ 11.59cm
    SetServoAngles(robot->theta[0], robot->theta[1], robot->theta[2]);
 }
 