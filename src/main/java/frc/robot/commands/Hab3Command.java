package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;
import frc.robot.subsystems.DriveSubsystem;

public class Hab3Command extends Command {
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2; 

    public Hab3Command() {
        requires(driveSubsystem);
        requires(armSubsystem);
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        armSubsystem.setShoulderJointSpeed(ConstantsMap.HAB_SHOULDER_RAISE_SPEED);
        // TODO: check that this is accurate
        armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle() - 180);
        driveSubsystem.setScrewVelocitySetpoint(armSubsystem.getShoulderSpeed());
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        driveSubsystem.setScrewVelocitySetpoint(0);
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        driveSubsystem.setScrewVelocitySetpoint(0);
    }
}
