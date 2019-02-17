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

    public static final int LEFT_DRIVE_PORT_1 = 3;
    public static final int LEFT_DRIVE_PORT_2 = 4;
    public static final int RIGHT_DRIVE_PORT_1 = 1;
    public static final int RIGHT_DRIVE_PORT_2 = 2;

   
    public static final int SHOULDER_JOINT_RIGHT_PORT = 6;
    public static final int SHOULDER_JOINT_LEFT_PORT = 5;
    public static final int WRIST_JOINT_PORT = 7;

    public static final int GRAB_SPIN_1 = 8;
    public static final int GRAB_SPIN_2 = 9;



    public static final int LOWER_ARM_LIMIT_PORT = 0;


    public static final int LOWER_WRIST_LIMIT_PORT = 2;
    public static final int LOWER_WRIST_LIMIT_PORT2 = 3;


    public static final int HATCH_PISTON_F = 4;
    public static final int HATCH_PISTON_R = 5;
    public static final int RELEASE_PISTON_1_F = 7;
    public static final int RELEASE_PISTON_1_R = 6;
}
