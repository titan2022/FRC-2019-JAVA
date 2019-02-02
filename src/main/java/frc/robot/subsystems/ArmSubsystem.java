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
    private Encoder shoulderEncoder, wristEncoder;

    private DigitalInput upperLimit, lowerLimit;
    

    public ArmSubsystem() {
        shoulderEncoder = new Encoder(RobotMap.SHOULDER_ENCODER_PORT_A, RobotMap.SHOULDER_ENCODER_PORT_B);
        wristEncoder = new Encoder(RobotMap.WRIST_ENCODER_PORT_A, RobotMap.WRIST_ENCODER_PORT_B);
        wristEncoder.reset();
        shoulderEncoder.reset();

        shoulderMotors = new TalonSet(new WPI_TalonSRX[] {new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_PORT_1), 
            new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_PORT_2)});
        
        wristMotor = new TalonSet(new WPI_TalonSRX(RobotMap.WRIST_JOINT_PORT));

        DigitalInput upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        DigitalInput lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
    }

    public Encoder getWristEncoder() {
        return wristEncoder;
    }

    public Encoder getShoulderEncoder() {
        return shoulderEncoder;
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

    public double getShoulderDistance() {
        return shoulderEncoder.getDistance();
    }

    public double getWristDistance() {
        return wristEncoder.getDistance();
    }

    public boolean getShoulderUpperLimit(){
        return upperLimit.get();
    }

    public boolean getShoulderLowerLimit(){
        return lowerLimit.get();
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
