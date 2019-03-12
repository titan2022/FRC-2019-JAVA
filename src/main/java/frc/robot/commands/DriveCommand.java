/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.ArmSubsystem2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.ConstantsMap;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveCommand extends Command {
	
	DriveSubsystem driveSubsystem = Robot.driveSubsystem;
	ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
	OI oi = Robot.oi;
	boolean turtlemode = false;	
	boolean brakeState = false;
	boolean rumble;
	long lastPressed = 0;

	
    public DriveCommand() {
		requires(driveSubsystem);
		
		//Set follow Line command booleans 
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Drive Comand init");
    	driveSubsystem.resetEncoders();
		driveSubsystem.resetGyro();
		//System.out.println(lineSubsystem.getData());	
		rumble = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {   
  

			double speedLeft = XboxMap.left();
			speedLeft *= -1;
			if(XboxMap.toggleTurtle()){
				turtlemode = !turtlemode;
			}
			if (Math.abs(speedLeft) < 0.1) {
				speedLeft = 0;
			}

			double speedRight = XboxMap.right();
			//speedRight *= -1;
			if(Math.abs(speedRight) < 0.1){
				speedRight = 0; 
			}
			if(turtlemode) {
				speedLeft *= ConstantsMap.TURTLE_SPEED;
				speedRight *= ConstantsMap.TURTLE_SPEED;
			}
			if(armSubsystem.getShoulderEncoderAngle()>0){
				speedLeft *= .75;
				speedRight *= .75;
			

			}
			
			driveSubsystem.setLeftSpeed(speedLeft);
			driveSubsystem.setRightSpeed(speedRight);

			//Auto Brake Mode
			//attack3Map.startAutoBrakerSystem();
			if(XboxMap.toggleBrakes()){  
				brakeState = !brakeState;
			}
			if(brakeState){
				driveSubsystem.enableBrake();
			}
			else if(!brakeState){
				driveSubsystem.disableBrake();
			}


			if(driveSubsystem.getVoltage()<6.8){
				lastPressed = System.currentTimeMillis();
				XboxMap.startRumble();
				rumble = true;
			}
			else if(rumble && ((System.currentTimeMillis()-lastPressed)>500)){
				XboxMap.stopRumble();
				rumble = false;
			}

		
		//Putting Data up
    	displayData();

    }

    protected void displayData(){
    	SmartDashboard.putNumber("Right Count",driveSubsystem.getRightEncoderCount());
		SmartDashboard.putNumber("LEft Count",driveSubsystem.getLeftEncoderCount());
		SmartDashboard.putNumber("Right Distance",driveSubsystem.getRightEncoderDistance());
		SmartDashboard.putNumber("LEft Disatance",driveSubsystem.getLeftEncoderDistance());
		SmartDashboard.putBoolean("Turtle", turtlemode);
		SmartDashboard.putBoolean("Brake Mode", brakeState);

    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    protected boolean getBrakeState(){
    	return brakeState;
    }
    // Called once after isFinished returns true
    protected void end() {
		driveSubsystem.stop();
		System.out.println("Drive Command Stopped");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	driveSubsystem.stop();
	}
}