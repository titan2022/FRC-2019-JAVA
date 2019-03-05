package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.maths.Vector2;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public GoToDistance() {
		requires(driveSubsystem);
		//Set follow Line command booleans 

    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("Drive To Distance init");
        double startAngle = SmartDashboard.getNumber("LineStartAngle", 0);
        double startDistance = SmartDashboard.getNumber("LineStartDistance", 0);
        double backAngle = SmartDashboard.getNumber("LineBackAngle", 0);
        double backDistance = SmartDashboard.getNumber("LineStartDistance", 0);
        Vector2 start = Vector2.fromPolar(startAngle, startDistance);
        Vector2 back = Vector2.fromPolar(backAngle, backDistance);
        Vector2 line = start.add(back.multiply(-1));   
        
        Vector2 extendedLine = back.proj(line);
        Vector2 target = start.add(extendedLine.multiply(-1));   
        distance = Math.sqrt(target.x * target.x + target.y + target.y); 
        System.out.println("Distance:" + distance);
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