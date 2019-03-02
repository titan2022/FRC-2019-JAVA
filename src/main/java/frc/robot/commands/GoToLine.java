package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;


import com.kauailabs.navx.frc.AHRS;


/**
 * Add your docs here.
 */
public class GoToLine extends Command {
    
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;

    AHRS gyro;

    //moar variables

    protected double estimatedDistanceToWall;

    public GoToLine() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("GoToLine init");
        gyro = driveSubsystem.getGyro();
        
    }

    
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("GoToLine end");
    }

    // Called for manual interruption of command
    protected void kill() {

        System.out.println("GoToLine kill");
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("GoToLine interrupted");
        kill();
    }
}