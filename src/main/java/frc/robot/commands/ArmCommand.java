/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
    EncoderMotorPID wristLevelPID;

    public ArmCommand() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        wristLevelPID = new EncoderMotorPID(armSubsystem.getWristEncoder(), new Motortronic(armSubsystem.getWristTalon()), ConstantsMap.WRIST_ZERO_KP,
            ConstantsMap.WRIST_ZERO_KI, ConstantsMap.WRIST_ZERO_KD, ConstantsMap.WRIST_ZERO_KF).setOutputRange(-1,1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double moveShoulderJoint = XboxMap.controlShoulderJoint();
        double moveWristJoint = XboxMap.controlWristJoint();

        wristLevelPID.setSetpoint(pidCurrentSP());
        // TODO: restructure w/ tolerance equality
        if ((armSubsystem.getWristDistance() != 0 && moveShoulderJoint != 0 && moveWristJoint == 0) ||
            XboxMap.enableZeroPid()) {
            wristLevelPID.enable();
        } else if (armSubsystem.getWristDistance() == 0 || moveWristJoint != 0 ) {
            wristLevelPID.disable();
        }

        if (wristLevelPID.isEnabled()) return;

        armSubsystem.setShoulderJointSpeed(ConstantsMap.SHOULDER_SPEED_MULT * moveShoulderJoint);
        armSubsystem.setWristJointSpeed(ConstantsMap.WRIST_SPEED_MULT * moveWristJoint);
    }

    private void setSpeed(double speed) {
        armSubsystem.setWristJointSpeed(speed);
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
        return 90.00 - armSubsystem.getShoulderEncoderAngle();
    }
}
