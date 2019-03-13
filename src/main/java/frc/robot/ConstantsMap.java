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
    public static final double TURTLE_SPEED = .6; //to be determined
    public static final double JOYSTICK_SENSITIVITY = .1; //to be determined

    public static final double TIP_TOLERANCE = 8;
    public static final double DRIFT_TOLERANCE = 5; 

    public static final Gains driveGains = new Gains(1, 0.0, 0, 0, 0, 1.0);
    public static final Gains turnGains = new Gains(.2, 0.0, .1, 0, 0, 1.0);
    public static final int DRIVE_VELOCITY = 3400;
    public static final int DRIVE_ACCEL = 3400;



    public static final double DRIVE_STALL = 10;
    public static final double DRIVE_STALL_TIME = 1;

    public static final double WRIST_STALL = 10;
    public static final double WRIST_STALL_TIME = 1;

    public static final double SHOULDER_STALL = 10;
    public static final double SHOULDER_STALL_TIME = 1;



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
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30; 

    

    public static final double WRIST_SPEED_MULT = .1;
    public static final double SHOULDER_SPEED_MULT = .1;
    public static final double WRIST_ENCODER_TICKS_PER_ROTATION = 4096;
    //public static final double WRIST_ENCODER_ANGLE_PER_TICK = 360/((double)WRIST_ENCODER_TICKS_PER_ROTATION * 63);
    public static final double WRIST_ENCODER_ANGLE_PER_TICK = 360/((double)WRIST_ENCODER_TICKS_PER_ROTATION * 100 * (38.0/18.0));

    public static final double WRIST_MIN_ANGLE_DOWN = -10.0;
    public static final double WRIST_MIN_ANGLE_UP = -60.0;

    //public static final double WRIST_MAX_ANGLE = 80.0;
    public static final double WRIST_MAX_ANGLE = 88.0;

    public static final double WRIST_IN_ANGLE = 60.0;

    public static final int WRIST_VELOCITY = 1000;
    public static final int WRIST_ACCEL = 1000;
    public static final int  WRIST_FOLLOW_DISTANCE = 20;
    public static final double WRIST_CHANGE_SETPOINT_SPEED = .2;

    public static final Gains wristGains = new Gains(0.01, 0.0, 0.0, 0.0, 0, 1.0);
    //public static final Gains wristGains = new Gains(0.2, 0.001, 0, .1, 20, 1.0);
    //public static final Gains wristGains = new Gains(0.21, 0.0005, 0, 0, 20, 1.0);


    public static final int SHOULDER_ENCODER_TICKS_PER_ROTATION = 4096; //ticks
    public static final double SHOULDER_ENCODER_ANGLE_PER_TICK = 360/((double)SHOULDER_ENCODER_TICKS_PER_ROTATION * 49 * 5);

    public static final double SHOULDER_GEAR_RATIO = 350.0;
    public static final double SHOULDER_MIN_ANGLE = -67.0;
    public static final double SHOULDER_MAX_ANGLE = 50.0;
    
    public static final int  SHOULDER_FOLLOW_DISTANCE = 20;
    public static final double SHOULDER_CHANGE_SETPOINT_SPEED = .2;
    public static final int  SHOULDER_WRIST_FOLD_ANGLE  = -65;

    public static final int SHOULDER_VELOCITY = 26000;
    public static final int SHOULDER_ACCEL = 24000;
    public static final Gains shoulderGains = new Gains(.2 , 0.0001, 0, 0.042625, 20, 1.0);
    //public static final Gains shoulderGains = new Gains(.2, 0.0, 0, 0.042625, 0, 1.0);

    //For Grabber
    public static final double BALL_INTAKE_SPEED = .5;
    public static final double BALL_OUTTAKE_SPEED = 1;



    //Follow line pid values
    public static final double PID_PERCENT_TOLERANCE = .01;
    public static final double PID_OUTPUT_MAX = .25;
    public static final double PID_PROPORTION_CONSTANT = 1;
    public static final double PID_INTEGRAL_CONSTANT = .2;
    public static final double PID_DERIVATIVE_CONSTANT = .001;
    public static final double PID_INPUT_MAX = 180;

    // PRESETS!!!
    public static final boolean isHatchGrabber = true;
    public static final double ROCKET_HATCH_PRESET_1 = -57;
    public static final double ROCKET_HATCH_PRESET_2 = -10.0;
    public static final double ROCKET_HATCH_PRESET_3 = 34;
    public static final double ROCKET_BALL_PRESET_1 = -35;
    public static final double ROCKET_BALL_PRESET_2 = 5;
    public static final double ROCKET_BALL_PRESET_3 = 46;

    public static final double CARGO_BALL_PRESET = -20.0;
    public static final double CARGO_HATCH_PRESET = -20.0;

    public static final double HATCH_COLLECT_PRESET = -57.0;
    public static final double BALL_COLLECT_WRIST_PRESET = -5.5;
    public static final double BALL_COLLECT_SHOULDER_PRESET = -47;

    
    public static final double GO_HOME_PRESET = SHOULDER_MIN_ANGLE;   


}