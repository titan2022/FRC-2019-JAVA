/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        TalonSRX rightShoulder, leftShoulder,wrist;
        rightShoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_RIGHT_PORT);
        leftShoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);
        rightShoulder.setInverted(true);
        shoulderMotors = new TalonSet(new TalonSRX[] {rightShoulder, leftShoulder})
            .setDistance(ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK);
        wrist = new TalonSRX(RobotMap.WRIST_JOINT_PORT);
        wrist.setInverted(true);
        wristMotor = new TalonSet(wrist)
            .setDistance(ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK);

        //upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        //lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        //upperLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimitWrist.setName("LowerWristLimit");
        SmartDashboard.putData(lowerLimitWrist);
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

    public void zeroWrist() {
        wristMotor.zeroEncoder();;
    }
    public void zeroShoulder() {
        shoulderMotors.zeroEncoder();;
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

        return !lowerLimitWrist.get();
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
