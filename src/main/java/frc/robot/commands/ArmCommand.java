package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.pids.EncoderMotorPID;
import frc.robot.pids.Motortronic;
import frc.robot.ConstantsMap;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    ArmSubsystem armSubsystem = Robot.armSubsystem;
    EncoderMotorPID wristLevelPID, armMovementPID;

    public ArmCommand() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        //Generalizes the interface between PID for each mechanism and its components.
        wristLevelPID = new EncoderMotorPID(armSubsystem.getWristEncoder(), armSubsystem.getWristMotortronic(), ConstantsMap.WRIST_ZERO_KP,
            ConstantsMap.WRIST_ZERO_KI, ConstantsMap.WRIST_ZERO_KD, ConstantsMap.WRIST_ZERO_KF).setOutputRange(-1,1);
        armMovementPID = new EncoderMotorPID(armSubsystem.getShoulderEncoder(), armSubsystem.getShoulderMotortronic(), ConstantsMap.SHOULDER_MV_KP,
            ConstantsMap.SHOULDER_MV_KI, ConstantsMap.SHOULDER_MV_KD, ConstantsMap.SHOULDER_MV_KF).setOutputRange(-1,1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double moveShoulderJoint = XboxMap.controlShoulderJoint();
        double moveWristJoint = XboxMap.controlWristJoint();

        //Where the wrist is currently, so the wrist will be held at the height that it is at when the joystick is released.
        wristLevelPID.setSetpoint(pidCurrentSP());

        //The PID should be enabled and going to the set point when:
        //The wrist is off the set point target
        //The driver isn't moving the wrist
        //The driver is moving the shoulder
        //The driver wants the wrist to be levelled
        if ((Math.abs(armSubsystem.getWristDistance()) > ConstantsMap.WRIST_TOLERANCE && Math.abs(moveShoulderJoint) > ConstantsMap.JOYSTICK_SENSITIVITY && Math.abs(moveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY) || XboxMap.enableZeroPid()) {
            wristLevelPID.enable();
        
        //The PID should be disabled when:
        //The wrist has reached the set point
        //The driver wants to move the wrist
        } else if ((Math.abs(armSubsystem.getWristDistance()) > ConstantsMap.WRIST_TOLERANCE) || Math.abs(moveWristJoint) > ConstantsMap.JOYSTICK_SENSITIVITY) {
            wristLevelPID.disable();
        }

        //When the PID is on it will get new targets for what angle the Encoder should be at, so that it is level
        if (wristLevelPID.isEnabled()) {
            wristLevelPID.setSetpoint(getWristLevelledAngle());
            return;
        }
        
        //THe driver wants the arm or shoulder to move, so it will move at the speed with the multiplier applied.
        armMovementPID.setSetpoint(armMovementPID.getSetpoint() + ConstantsMap.SHOULDER_SPEED_MULT * moveShoulderJoint);
        armSubsystem.setWristJointSpeed(ConstantsMap.WRIST_SPEED_MULT * moveWristJoint);
    }

    /**
     * Computes the angle of the shoulder as a ratio of the motors to the arm. 
     * 0 degrees is straight down and positive is infront of the robot.
     * @return angle in degrees
     */
    private double getShoulderEncoderAngle() {
        return ConstantsMap.SHOULDER_OFFSET + armSubsystem.getShoulderDistance() / 350.0;
    }

    /**
     * Gets the angle relative to the encoder on the wrist that the wrist should go to be level.
     * @return
     */
    private double getWristLevelledAngle() {
        return 180 - (90 - getShoulderEncoderAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

    public double pidCurrentSP() {
        return 90.00 - getShoulderEncoderAngle();
    }
}