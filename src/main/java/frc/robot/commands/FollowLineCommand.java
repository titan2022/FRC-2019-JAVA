/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.subsystems.FollowLineSubsystem;
import java.lang.Math.*;

/**
 * Add your docs here.
 */
public class FollowLineCommand extends Command {
    FollowLineSubsystem followLineSubsystem = Robot.followLineSubsystem;

    protected boolean visionStageComplete = false;
    protected boolean oneSensorStageComplete = false;
    protected boolean twoSensorStageComplete = false;

    protected double estimatedDistanceToWall;

    public FollowLineCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.subsystem);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("FollowLineCommand init");
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        System.out.println("FollowLineCommand execute");

        if(!visionStageComplete){
            visionStage();
        } else if(!oneSensorStageComplete){
            oneSensorStage();
        } else if(!twoSensorStageComplete){
            twoSensorStage();
        } else{
            //we are finished, idk what to do
        }

    }

    // Called by execute to approach the tape using vision
    protected void visionStage () {
        //i have no clude as of yet, this is a different problem
        visionStageComplete = true;
    }
    
    // Called by execute to line up when only 1 sensor has seen tape
    protected void oneSensorStage () {
        boolean[] isFrontCameraOnStrip = followLineSubsystem.getCameraData(1); 
        boolean[] isBackCameraOnStrip = followLineSubsystem.getCameraData(2);

        for(boolean b: isBackCameraOnStrip){
            if(b){
                oneSensorStageComplete = true;
                return;
            }
        }

        //actually implement the turning and math
    }

    // Called by execute to line up when both sensors see tape
    protected void twoSensorStage () {

        //this is so we know when to stop, will be uncommented when we have a mechanism for detecting when we hit the wall
        /*if(we hit the wall)
        {
            twoSensorStageComplete = true;
        }*/

        double frontAverage = followLineSubsystem.getCameraAverage(1);
        double backAverage = followLineSubsystem.getCameraAverage(2);

        double diff = backAverage - frontAverage;

        double theta = Math.acos(diff / Math.sqrt(diff * diff + ConstantsMap.DISTANCE_BETWEEN_SENSORS * ConstantsMap.DISTANCE_BETWEEN_SENSORS));

        double leftWheelDistance = estimatedDistanceToWall + theta * ConstantsMap.ROBOT_WIDTH;
        double rightWheelDistance = estimatedDistanceToWall - theta * ConstantsMap.ROBOT_WIDTH;
        
        //actually implement the turning
    } 
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("FollowLineCommand end");
    }

    // Called for manual interruption of command
    protected void kill(){

    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("FollowLineCommand interrupted");
    }
}