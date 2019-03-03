package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GoToDistance extends Command {
	
	DriveSubsystem driveSubsystem = Robot.driveSubsystem;
	double distance;


	
    public GoToDistance(double distance) {
		requires(driveSubsystem);
		this.distance = distance;
		//Set follow Line command booleans 

    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Drive To Distance init");
    	driveSubsystem.resetEncoders();
		driveSubsystem.resetGyro();
		driveSubsystem.setTravel(distance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {   
        
	}
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(driveSubsystem.getLeftEncoderDistance()-distance)<1;
    }


    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Drive To Distance End");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        driveSubsystem.stop();
        System.out.println("Drive To Distance Interrupt");
	}
}