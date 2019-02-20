package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;

public class GoHome extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;

    public GoHome() {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {        
        
        armSubsystem.setShoulderSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);
        armSubsystem.setWristSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);


    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {        

        //armSubsyste

    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        //return false;

            return Math.abs(armSubsystem.getShoulderEncoderAngle() - ConstantsMap.SHOULDER_MIN_ANGLE)<1;

        
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
