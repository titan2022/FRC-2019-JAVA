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
    public static final double JOYSTICK_SENSITIVITY = 1; //to be determined


    //for line follower
    public static final double DISTANCE_BETWEEN_SENSORS = 6; //inches
    public static final double DISTANCE_BETWEEN_SENSOR_CAMERAS = .5;//inches 
    public static final double ROBOT_WIDTH = 26; //inches
    public static final double ROBOT_LENGTH = 32; //inches
    public static final double APPROACH_TIME = 1.0d; //seconds
    public static final double APPROACH_SPEED = .12; //inches per second
    public static final double ANGLE_TOLLERANCE = 0.25; //degrees
    public static final double TURN_SPEED = .5; //inches per second
    public static final double BLACK_WHITE_CUTOFF = 220; //some units idk
    public static final double SENSOR_AVERAGE_TOLERANCE_HIGH = 1.0d;
    public static final double SENSOR_AVERAGE_TOLERANCE_LOW = 0.1d;
    public static final double SENSOR_AVERAGE_CENTER = 3.5d;
    public static final double VISION_THRESHOLD = 0.5; //degrees

    public static final double LOW_CUTOFF = 210;
    public static final double HIGH_CUTOFF = 230;

    //For ArmSubsystem
    public static final double WRIST_ZERO_KP = 1;
    public static final double WRIST_ZERO_KI = 1;
    public static final double WRIST_ZERO_KD = 1;
    public static final double WRIST_ZERO_KF = 0;

    public static final double WRIST_SPEED_MULT = .5;
    public static final double SHOULDER_SPEED_MULT = .1;
    public static final double WRIST_ENCODER_TICKS_PER_ROTATION = 4096;

    public static final int SHOULDER_ENCODER_TICKS_PER_ROTATION = 4096; //ticks
    public static final double SHOULDER_GEAR_RATIO = 350.0;
    public static final double SHOULDER_MIN_ANGLE = 23.0;
    public static final double SHOULDER_MAX_ANGLE = 121.0;

    public static final double SHOULDER_MV_KP = 1.0;
    public static final double SHOULDER_MV_KI = 1.0;
    public static final double SHOULDER_MV_KD = 1.0;
    public static final double SHOULDER_MV_KF = 1.0;

    public static final double SHOULDER_OFFSET = 23.5; //to be changed if needed
    public static final double WRIST_OFFSET = 0; //to be changed

    //Tolerances
    public static final double WRIST_TOLERANCE = Float.NaN; //to be determined
    public static final double SHOULDER_TOLERANCE = Float.NaN; //to be determined

    //Follow line pid values
    public static final double PID_PERCENT_TOLERANCE = .01;
    public static final double PID_OUTPUT_MAX = .25;
    public static final double PID_PROPORTION_CONSTANT = 1;
    public static final double PID_INTEGRAL_CONSTANT = .2;
    public static final double PID_DERIVATIVE_CONSTANT = .001;
    public static final double PID_INPUT_MAX = 180;
}