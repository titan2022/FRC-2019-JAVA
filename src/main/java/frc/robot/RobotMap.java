/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static final int LEFT_DRIVE_PORT_1 = 0;
    public static final int LEFT_DRIVE_PORT_2 = 1;
    public static final int RIGHT_DRIVE_PORT_1 = 2;
    public static final int RIGHT_DRIVE_PORT_2 = 3;

    public static final int LEFT_ENCODER_PORT_A = 4;
    public static final int LEFT_ENCODER_PORT_B = 5;

    public static final int RIGHT_ENCODER_PORT_A = 6;
    public static final int RIGHT_ENCODER_PORT_B = 7;

    public static final int SHOULDER_JOINT_PORT_1 = 8;
    public static final int SHOULDER_JOINT_PORT_2 = 9;
    public static final int WRIST_JOINT_PORT = 10;

    public static final int SHOULDER_ENCODER_PORT_A = 11;
    public static final int SHOULDER_ENCODER_PORT_B = 12;

    public static final int SHOULDER_ENCODER_PORT_C = 420;
    public static final int SHOULDER_ENCODER_PORT_D = 420;

    public static final int WRIST_ENCODER_PORT_A = 13;
    public static final int WRIST_ENCODER_PORT_B = 14;

    public static final int UPPER_ARM_LIMIT_PORT = 15;
    public static final int LOWER_ARM_LIMIT_PORT = 16;

    //TODO Set these to real ports 
    public static final int UPPER_WRIST_LIMIT_PORT = 0;
    public static final int LOWER_WRIST_LIMIT_PORT = 0;
}
