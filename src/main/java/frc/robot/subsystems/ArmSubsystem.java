/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ConstantsMap;
import frc.robot.RobotMap;
import frc.robot.TalonSet;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private TalonSet shoulderMotors, wristMotor;
    private Encoder shoulderEncoder, wristEncoder;

    private DigitalInput upperLimit, lowerLimit, upperLimitWrist, lowerLimitWrist;
    
    private TalonSRX rightShoulder, leftShoulder;

    public ArmSubsystem() {
        //shoulderEncoder = new Encoder(RobotMap.SHOULDER_ENCODER_PORT_A, RobotMap.SHOULDER_ENCODER_PORT_B);
        //wristEncoder = new Encoder(RobotMap.WRIST_ENCODER_PORT_A, RobotMap.WRIST_ENCODER_PORT_B);
       // wristEncoder.reset();
        //shoulderEncoder.reset();

        rightShoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_RIGHT_PORT);
        leftShoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);
        rightShoulder.setInverted(true);
        shoulderMotors = new TalonSet(new TalonSRX[] {rightShoulder, leftShoulder});
        
        wristMotor = new TalonSet(new TalonSRX(RobotMap.WRIST_JOINT_PORT));

        ///upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        //lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        //upperLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        //lowerLimitWrist = new DigitalInput(RobotMap.UPPER_WRIST_LIMIT_PORT);
    }

    public double getShoulderEncoderAngle()
    {
        return 360 * getShoulderTalons().encTicks() * 1.0 / ConstantsMap.SHOULDER_ENCODER_TICKS_PER_ROTATION;
    }

    public double getWristEncoderAngle()
    {
        return 360 * getWristTalons().encTicks() * 1.0 / ConstantsMap.SHOULDER_ENCODER_TICKS_PER_ROTATION;
    }

    public TalonSet getWristTalons() {
        return wristMotor;
    }

    public TalonSet getShoulderTalons() {
        return shoulderMotors;
    }

    public void setShoulderJointSpeed(double speed) {
        shoulderMotors.set(speed);
    }

    public void setWristJointSpeed(double speed) {
        wristMotor.set(speed);
    }

    public boolean getShoulderUpperLimit(){
        return upperLimit.get();
    }

    public boolean getShoulderLowerLimit(){
        return lowerLimit.get();
    }

    public boolean getWristUpperLimit(){
        return upperLimitWrist.get();
    }

    public boolean getWristLowerLimit(){
        return lowerLimitWrist.get();
    }

    public double getShoulderSpeed(){
        return shoulderMotors.get();
    }

    public double getWristSpeed(){
        return wristMotor.get();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
