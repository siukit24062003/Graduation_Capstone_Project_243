/**
 * ball_and_plate.h
 * Định nghĩa API chính cho hệ thống ball and plate
 */
#ifndef BALL_AND_PLATE_H
#define BALL_AND_PLATE_H

#include "robot_geometry.h"
#include "inverse_kinematics.h"
#include "servo_control.h"

/* Prototype các hàm chính */
void RobotInit(RobotState *robot);
void PerformInitialMovements(RobotState *robot);

#endif /* BALL_AND_PLATE_H */
