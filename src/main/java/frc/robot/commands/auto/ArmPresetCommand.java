package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;

public class ArmPresetCommand extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    double gotoAngle;

    public ArmPresetCommand(double targetAngle) {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);
        gotoAngle = targetAngle;
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        armSubsystem.setShoulderSetPoint(gotoAngle);
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle());
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // TODO: will this work or should we use tolerance equality?
        return armSubsystem.getShoulderEncoderAngle() == gotoAngle;
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
}
