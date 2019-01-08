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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FollowLineSubsystem;
import java.lang.Math;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class FollowLineCommand extends Command {
    FollowLineSubsystem followLineSubsystem = Robot.followLineSubsystem;
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;

    protected boolean visionStageComplete;
    protected boolean oneSensorStageComplete;
    protected boolean twoSensorStageComplete;

    //stuff for stage 2
    ArrayList<Double> oSSPreviousAverages;
    int numOfJumps; 
    int[] jumpIndices; // these will be the index after the change
    int[] jumpEncoderCount; // relative to the start of stage 2
    long startTime;

    //Stuff for stage one 
    AHRS gyro;


    protected double estimatedDistanceToWall;

    public FollowLineCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.subsystem);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("FollowLineCommand init");
        setupForRun();
    }

    //this is to be called upon initialization and whenever the button is hit twice
    protected void setupForRun(){

        visionStageComplete = false;
        oneSensorStageComplete = false;
        twoSensorStageComplete = false;
    
        oSSPreviousAverages = new ArrayList<Double>();
        numOfJumps = 0; 
        jumpIndices = new int[]{0,0};
        jumpEncoderCount = new int[]{0, 0};
        startTime = System.nanoTime();
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

        //TODO: use vision somehow....
        //i have no clue as of yet, this is a different problem for somebody else

        //although, i will say that hopefully we can make so that even if we approach the wall at some terrible angle, then we can hopefully use vision
        //and motion profiling in conjunction to make the robot sort of swing outward to approach from a more reasonable angle, because if we approach 
        //from an angle of incedence to the wall of less than say 25 degrees, steps 2 and three WILL fail to fix it.
        //any questions or ideas on how to do this better, talk to me(jake).
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

        //if(oSSPreviousAverages.size())

        //okay so here is my idea, we should use the arraylist of previous whatever to determine the trend of where we are going(this is to be 
        //done by mesasuing the time between jumps, as the number is a step function). once we see a change,we can then calculate the angle off that we
        //are, then use the gyro to get our current angle and then calculate the new angle. Once we have our desired angle, we adjust for the swing of
        //the front of the robot about its center iwth forward movement and only then do we we orient ourselves to the desired angle. Then once the back
        //sensor hits, we can do the fine tuning. 
        //
        //do we even have a gyro on this bot? hopefully? if not we can calulate it using the change in positoin of left and right wheels and calculate 
        //the angle, but that is so messy
        //
        //i *try* to implement this below
        //
        //any questions or ideas on how to do this better, talk to me(jake).

        //this gets the jumps in average
        if(numOfJumps < 2)
        {
            double frontAverage = followLineSubsystem.getCameraAverage(1);
            oSSPreviousAverages.add(frontAverage);
            int oSSSize = oSSPreviousAverages.size();

            if(oSSSize > 1 && numOfJumps < 2){//just making sure we dont step outside oSSpreviousaverages
                if(Math.abs(oSSPreviousAverages.get(oSSSize - 1) - oSSPreviousAverages.get(oSSSize)) > 0.1){
                    //now we have a step

                    jumpIndices[numOfJumps] = oSSSize - 1;
                    jumpEncoderCount[numOfJumps] = 0; //TODO: get encoder values here, maybe average left and right? // yes divide by 1 billion

                    numOfJumps++;
                }
            }
            return; // please note this return when considering flow of this function
        }
        
        //start to use ith
        double deltaDistance = (jumpEncoderCount[1] - jumpEncoderCount[0]) *  ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
        double deltaAverage = oSSPreviousAverages.get(jumpIndices[1]) - oSSPreviousAverages.get(jumpIndices[0] - 1);
        //double distanceTraveled = 

        
        //TODO: actually implement the turning and math
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

        //TODO: CONVERT THE DIFF TO ACTUAL UNITS(INCHES), NOT JUST OUR ARBITRARY ONES. THIS IS VERY VERY IMPORTANT. i would now, but i need measurements

        //get the error angle
        double theta = Math.acos(diff / Math.sqrt(diff * diff + ConstantsMap.DISTANCE_BETWEEN_SENSORS * ConstantsMap.DISTANCE_BETWEEN_SENSORS));

        //use the angle to figure out how far the wheels must correct to appraoch the wall so that it is perpedicular.
        //note: this method is not perfect, but rather an approximation of the best way to correct our angle as it makes us follow an arc, meanding we 
        //become off center. But the better lined up we are before this step, the less we will shift side to side due to an arc(think about it, the error
        //is almsot exactly equal to 1-cos(error angle)) in one direction or the other. 
        double leftWheelDistance = estimatedDistanceToWall + theta * ConstantsMap.ROBOT_WIDTH;
        double rightWheelDistance = estimatedDistanceToWall - theta * ConstantsMap.ROBOT_WIDTH;

        //these units should be in INCHES
        double leftWheelSpeed = leftWheelDistance / ConstantsMap.APPROACH_TIME;
        double rightWheelSpeed = rightWheelDistance / ConstantsMap.APPROACH_TIME;

        //finally implement the turning/movement
        driveSubsystem.setLeftSpeed(leftWheelSpeed);
        driveSubsystem.setRightSpeed(rightWheelSpeed);
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