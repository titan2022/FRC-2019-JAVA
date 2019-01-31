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
import frc.robot.pids.Motortronic;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private WPI_TalonSRX shoulderJointMotor1, shoulderJointMotor2, wristJointMotor;
    private Encoder shoulderEncoder, wristEncoder;

    private DigitalInput upperLimit, lowerLimit;
    

    public ArmSubsystem() {
        shoulderJointMotor1 = new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_PORT_1);
        shoulderJointMotor2 = new WPI_TalonSRX(RobotMap.SHOULDER_JOINT_PORT_2);
        shoulderEncoder = new Encoder(RobotMap.SHOULDER_ENCODER_PORT_A, RobotMap.SHOULDER_ENCODER_PORT_B);

        wristJointMotor = new WPI_TalonSRX(RobotMap.WRIST_JOINT_PORT);
        wristEncoder = new Encoder(RobotMap.WRIST_ENCODER_PORT_A, RobotMap.WRIST_ENCODER_PORT_B);
        wristEncoder.reset();
        shoulderEncoder.reset();

        DigitalInput upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        DigitalInput lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
    }

    public Encoder getWristEncoder() {
        return wristEncoder;
    }

    public Encoder getShoulderEncoder() {
        return shoulderEncoder;
    }

    public Motortronic getWristMotortronic() {
        return new Motortronic(wristJointMotor);
    }

    public Motortronic getShoulderMotortronic() {
        return new Motortronic(new WPI_TalonSRX[] {shoulderJointMotor1, shoulderJointMotor2});
    }

    public void setShoulderJointSpeed(double speed) {
        shoulderJointMotor1.set(speed);
        shoulderJointMotor2.set(speed);
    }

    public void setWristJointSpeed(double speed) {
        wristJointMotor.set(speed);
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
        return shoulderJointMotor1.get();
    }

    public double getWristSpeed(){
        return wristJointMotor.get();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
