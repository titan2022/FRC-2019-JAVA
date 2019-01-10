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
    public static final double ROBOT_WHEEL_RADIUS_INCHES = 2; //inches
    public static final double DRIVE_ENCODER_TICKS_PER_ROTATION = 4096; //ticks
    public static final double DRIVE_ENCODER_DIST_PER_TICK = ((ROBOT_WHEEL_RADIUS_INCHES * 2 * Math.PI)/(DRIVE_ENCODER_TICKS_PER_ROTATION)); //inches 
    public static final double TURTLE_SPEED = .5; //to be determined

    //for line follower
    public static final double DISTANCE_BETWEEN_SENSORS = 6; //inches
    public static final double DISTANCE_BETWEEN_SENSOR_CAMERAS = .5;//inches 
    public static final double ROBOT_WIDTH = 26; //inches
    public static final double ROBOT_LENGTH = 32; //inches
    public static final double APPROACH_TIME = 1.0d; //seconds
    public static final double STAGE_TWO_SPEED = .12; //inches per second
    public static final double ANGLE_TOLLERANCE = 0.25; //degrees
    public static final double TURN_SPEED = .5; //inches per second
    public static final double BLACK_WHITE_CUTOFF = 220; //some units idk

    public static final double LOW_CUTOFF = 210;
    public static final double HIGH_CUTOFF = 230;
}