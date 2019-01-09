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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FollowLineSubsystem;
import frc.robot.ConstantsMap;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveCommand extends Command {
	
	DriveSubsystem driveSubsystem = Robot.driveSubsystem;
	FollowLineCommand fLC;
	OI oi = Robot.oi;
	boolean turtlemode = false;	
	boolean brakeState = false;
	long lastPressed = 0;

	//Things for follow line command
	private boolean runFollowLineCommand;

	
    public DriveCommand() {
		requires(driveSubsystem);
		
		//Set follow Line command booleans 
		runFollowLineCommand = false;
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Drive Comand init");
    	driveSubsystem.resetEncoders();
		driveSubsystem.resetGyro();
		//System.out.println(lineSubsystem.getData());	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {   
    	// Normal Driving
    	// if(attack3Map.turtleButton()) {
    	//	turtlemode = !turtlemode;
		// } 
		if (runFollowLineCommand) {
			if (!fLC.isFinished()) {
				if (XboxMap.interruptFollowLine()) {
					fLC.kill();//Make this a thing
				} else {
					fLC.execute();
				}
			} else {
				runFollowLineCommand = false;
			}
		} else {
			double speedLeft = XboxMap.left();
			speedLeft *= -1;

			if (Math.abs(speedLeft) < 0.1) {
				speedLeft = 0;
			}
			// if(xboxMap.shiftHigh()) {
			// 	driveSubsystem.shiftHigh();
			// }
			// if(xboxMap.shiftLow()) {
			// 	driveSubsystem.shiftLow();
			// }
			double speedRight = XboxMap.right();
			//speedRight *= -1;
			if(Math.abs(speedRight) < 0.1){
				speedRight = 0; 
			}
			if(turtlemode) {
				speedLeft *= ConstantsMap.TURTLE_SPEED;
				speedRight *= ConstantsMap.TURTLE_SPEED;
			}
			driveSubsystem.setLeftSpeed(speedLeft*ConstantsMap.TURTLE_SPEED);
			driveSubsystem.setRightSpeed(speedRight*ConstantsMap.TURTLE_SPEED);

			//Auto Brake Mode
			//attack3Map.startAutoBrakerSystem();
			if(XboxMap.startAutoBrakerSystem() && (System.currentTimeMillis() - lastPressed) > 200){  
				brakeState = !brakeState;
				lastPressed = System.currentTimeMillis();
			}
			if(brakeState){
				driveSubsystem.enableBrake();
			}
			else if(!brakeState){
				driveSubsystem.disableBrake();
			}

			//Check for follow line command call
			if (XboxMap.runFollowLineCommand()) {
				runFollowLineCommand = true;
				fLC = new FollowLineCommand();
			}
		}
		
		//Putting Data up
    	displayData();

    }

    protected void displayData(){
    	SmartDashboard.putNumber("Left Encoder Count: ", driveSubsystem.getLeftEncoderCount());
    	SmartDashboard.putNumber("Left Encoder Distance: ", driveSubsystem.getLeftEncoderDistance());
    	SmartDashboard.putNumber("Left Encoder Rate: ", driveSubsystem.getLeftEncoderRate());
    	SmartDashboard.putNumber("Right Encoder Count: ", driveSubsystem.getRightEncoderCount());
    	SmartDashboard.putNumber("Right Encoder Distance: ", driveSubsystem.getRightEncoderDistance());
    	SmartDashboard.putNumber("Right Encoder Rate: ", driveSubsystem.getRightEncoderRate());
    	SmartDashboard.putNumber("Gyro Angle: ", driveSubsystem.getGyroAngle());
    	SmartDashboard.putBoolean("AutoBrake", brakeState);
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	driveSubsystem.stop();
	}
}