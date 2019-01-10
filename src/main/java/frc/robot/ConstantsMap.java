/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class ConstantsMap {
    public static final double ROBOT_WHEEL_RADIUS_INCHES = Float.NaN; //inches
    public static final double DRIVE_ENCODER_TICKS_PER_ROTATION = 256; //ticks
    public static final double DRIVE_ENCODER_DIST_PER_TICK = ((ROBOT_WHEEL_RADIUS_INCHES * 2 * Math.PI)/(DRIVE_ENCODER_TICKS_PER_ROTATION)); //inches 
    public static final double TURTLE_SPEED = Float.NaN; //to be determined

    //for line follower
    public static final double DISTANCE_BETWEEN_SENSORS = 6; //inches
    public static final double ROBOT_WIDTH = 26; //inches
    public static final double ROBOT_LENGTH = 32; //inches
    public static final double APPROACH_TIME = 1.0d; //seconds

    //For ArmSubsystem
    public static final double WRIST_ZERO_KP = 1;
    public static final double WRIST_ZERO_KI = 1;
    public static final double WRIST_ZERO_KD = 1;
    public static final double WRIST_ZERO_KF = 0;

    public static final double WRIST_SPEED_MULT = .5;
    public static final double SHOULDER_SPEED_MULT = .5;

    public static final int SHOULDER_ENCODER_TICKS_PER_ROTATION = 420; //ticks

    public static final double SHOULDER_GEAR_RATIO = 420;
}