/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ConstantsMap;
import frc.robot.RobotMap;
import frc.robot.TalonSet;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private TalonSet shoulderMotors, wristMotor;


    private DigitalInput upperLimit, lowerLimit, upperLimitWrist, lowerLimitWrist;
    

    public ArmSubsystem() {
        WPI_TalonSRX rightShoulder, leftShoulder;
        rightShoulder = new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_RIGHT_PORT);
        leftShoulder = new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);
        rightShoulder.setInverted(true);
        shoulderMotors = new TalonSet(new WPI_TalonSRX[] {rightShoulder, leftShoulder})
            .setDistance(360 / ConstantsMap.WRIST_ENCODER_TICKS_PER_ROTATION);
        wristMotor = new TalonSet(new WPI_TalonSRX(RobotMap.WRIST_JOINT_PORT)).setDistance(360 / ConstantsMap.WRIST_ENCODER_TICKS_PER_ROTATION);

        //upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        //lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        //upperLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimitWrist = new DigitalInput(RobotMap.UPPER_WRIST_LIMIT_PORT);
    }

    public double getShoulderEncoderAngle() {
        return shoulderMotors.getEncoderDistance();
    }

    public double getWristEncoderAngle() {
        return wristMotor.getEncoderDistance();
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
