package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.ArmSubsystem2;

public class ArmPresetCommand extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    double preset;
    public ArmPresetCommand(double angle) {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);
        preset = angle;
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Start Preset");
        armSubsystem.setShoulderSetPoint(armSubsystem.getShoulderEncoderAngle());
        armSubsystem.setWristSetPoint(armSubsystem.getWristEncoderAngle());
        armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle());       
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {        
        
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Math.abs(armSubsystem.getShoulderEncoderAngle() - preset)<1;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("Preset Finished at: " + armSubsystem.getShoulderEncoderAngle() + " degress");

    }    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("Preset Interupt");
    }
}
