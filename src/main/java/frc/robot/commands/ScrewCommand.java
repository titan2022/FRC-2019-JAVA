package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.ControlPanelMap;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;
import frc.robot.subsystems.DriveSubsystem;

public class ScrewCommand extends Command {
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2; 

    public ScrewCommand() {
        requires(armSubsystem);
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }
    @Override
    protected void execute() {
        driveSubsystem.setScrewVelocitySetpoint(ConstantsMap.SCREW_SPEED * ControlPanelMap.controlShoulderJoint());

    }
    // Make this return true when this Command no longer needs to run execute()
    @Override
        protected boolean isFinished() {
        return Robot.debugMode && ControlPanelMap.stopClimb();
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        driveSubsystem.stopScrew();
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        driveSubsystem.stopScrew();
    }
}
