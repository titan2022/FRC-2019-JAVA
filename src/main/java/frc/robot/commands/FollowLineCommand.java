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
import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class FollowLineCommand extends Command {
    FollowLineSubsystem followLineSubsystem = Robot.followLineSubsystem;
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;

    //stage one variables (Vision Stage)
    protected boolean stageOneComplete;

    //stage two variables
    protected boolean stageTwoComplete;
    double leftEncoderDistanceGoal;
    double rightEncoderDistanceGoal;

    //stage three variables
    protected boolean stageThreeComplete;

    ArrayList<Double> oSSPreviousAverages;
    int numOfJumps; 
    int[] jumpIndices; // these will be the index after the change
    int[] jumpEncoderCount; // relative to the start of stage 2
    long startTime;

    //stage four variables
    protected boolean stageFourComplete;
    protected boolean stageFourFristRun;

    protected double deltaAverage;
    protected double deltaDistance;
    protected double deltaHorizontalDistance;
    protected double angleToLine;
    protected double desiredAngle;
    protected double swingDistance;
    protected double encoderGoal;

    //stage five variables
    protected boolean stageFiveComplete;

    //stage six variables
    protected boolean stageSixComplete;

    //Maybe this will work variables 
    protected double startEncoderAvg;
    protected double encoderFinalGoal;

    protected boolean firstRun;
    //Other stuff that we need 
    AHRS gyro;

    //moar variables
    private boolean overCompensate = false;

    protected double estimatedDistanceToWall;

    public FollowLineCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.subsystem);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("FollowLineCommand init");
        gyro = driveSubsystem.getGyro();
        setupForRun();
    }

    //this is to be called upon initialization and whenever the button is hit twice
    protected void setupForRun() { 
        //setup stage 1 variables
        stageOneComplete = false;

        //stage 2
        stageTwoComplete = false;
        leftEncoderDistanceGoal = 0;
        rightEncoderDistanceGoal = 0;

        //stage 3
        stageThreeComplete = false;

        oSSPreviousAverages = new ArrayList<Double>();
        numOfJumps = 0; 
        jumpIndices = new int[]{0,0};
        jumpEncoderCount = new int[]{0, 0};
        startTime = System.nanoTime();

        //stage 4
        stageFourComplete = false;
        
        //stage 5
        stageFiveComplete = false;

        //stage 6
        stageSixComplete = false;   
        
        //maybe 
        firstRun = true;
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        

        /*if(limit switch hit){
            setupForRun(    );//reset for next run
            somehow mark that we are done, maybe interrrupt?
        }else{*/
        // if(!stageOneComplete) {
        //     stageOne();
        // } else if(!stageTwoComplete) {
        //     stageTwo();
        // } else if(!stageThreeComplete) {
        //     stageThree();
        // } else if(!stageFourComplete) {
        //     stageFour();
        // } else if(!stageFiveComplete) {
        //     stageFive();
        // } else if(!stageSixComplete) {
        //     stageSix();
        // } 
<<<<<<< HEAD

        maybeThisWillWorkButIDRK();
    }

    protected void vision(){

        double horizontalAngle = 0;
        double verticalAngle = 0;

        if(horizontalAngle > ConstantsMap.VISION_THRESHOLD){
            //turn left
        }else if(horizontalAngle < ConstantsMap.VISION_THRESHOLD){
            //turn right
        } else{
            //go straight
        }
    }

=======
        
        approach();
    }
    
    /*DELETE THIS AT SOME POINT LATER WHEN OTHER METHODS WORK 
>>>>>>> f8e92135e9676795fff5d4989b69e53ca011de39
    //vision stage, get us closer to the line
    protected void stageOne() {
        System.out.println("stage 1");
        
        //TODO: use vision somehow....
        //i have no clue as of yet, this is a different problem for somebody else although, i will say that hopefully we can make so that even if we approach the wall at some terrible angle, then we can hopefully use vision and motion profiling in conjunction to make the robot sort of swing outward to approach from a more reasonable angle, because if we approach from an angle of incedence to the wall of less than say 25 degrees, steps 2 and three WILL fail to fix it. any questions or ideas on how to do this better, talk to me(jake).
        stageOneComplete = true;
    }

    //once we hit the line, make sure we go forward 2 inches
    protected void stageTwo() {
        //stageTwoComplete = true;

        System.out.println("stage 2");
        //set target
        if (leftEncoderDistanceGoal == 0 && rightEncoderDistanceGoal == 0) {
            leftEncoderDistanceGoal = driveSubsystem.getLeftEncoderDistance() + 2;
            rightEncoderDistanceGoal = driveSubsystem.getRightEncoderDistance() + 2;

            driveSubsystem.enableBrake();
        }

        driveSubsystem.setRightSpeed(ConstantsMap.APPROACH_SPEED);
        driveSubsystem.setLeftSpeed(ConstantsMap.APPROACH_SPEED);

        if ((driveSubsystem.getLeftEncoderDistance() > leftEncoderDistanceGoal)||(driveSubsystem.getRightEncoderDistance() > rightEncoderDistanceGoal)) {
            stageTwoComplete = true;
        }
    }

    //once we are over the line, then watch for the horizontal change
    protected void stageThree() {
        System.out.println("stage 3");
        //boolean[] isFrontCameraOnStrip = followLineSubsystem.getLineData(1); 
        //boolean[] isBackCameraOnStrip = followLineSubsystem.getLineData(2);

        // for(boolean b: isBackCameraOnStrip){
        //     if(b){ // jump to stage 6
        //         stageTwoComplete = true;
        //         stageThreeComplete = true;
        //         stageFourComplete = true;
        //         stageFiveComplete = true;
        //         return;
        //     }
        // }

        //if(oSSPreviousAverages.size())

        //okay so here is my idea, we should use the arraylist of previous whatever to determine the trend of where we are going(this is to be done by mesasuing the time between jumps, as the number is a step function). once we see a change,we can then calculate the angle off that we are, then use the gyro to get our current angle and then calculate the new angle. Once we have our desired angle, we adjust for the swing of the front of the robot about its center iwth forward movement and only then do we we orient ourselves to the desired angle. Then once the back sensor hits, we can do the fine tuning. 
        //do we even have a gyro on this bot? hopefully? if not we can calulate it using the change in positoin of left and right wheels and calculate 
        //the angle, but that is so messy
        //i *try* to implement this below
        //any questions or ideas on how to do this better, talk to me(jake).

        //this gets the jumps in average
        if(numOfJumps < 2) {
            double frontAverage = followLineSubsystem.getLineAverage(1);
            oSSPreviousAverages.add(frontAverage);
            int oSSSize = oSSPreviousAverages.size();

            if(oSSSize > 1 && numOfJumps < 2) {//just making sure we dont step outside oSSpreviousaverages
                if(Math.abs(oSSPreviousAverages.get(oSSSize - 1) - oSSPreviousAverages.get(oSSSize - 2)) > 0.1){
                    //now we have a step, so we can set our values

                    jumpIndices[numOfJumps] = oSSSize - 1;
                    jumpEncoderCount[numOfJumps] = (driveSubsystem.getLeftEncoderCount() + driveSubsystem.getRightEncoderCount()) / 2; 
                    
                    System.out.println("" + numOfJumps + ", " + jumpIndices[numOfJumps] + ", " + jumpEncoderCount[numOfJumps]);

                    numOfJumps++;
                }
            }
            return; // please note this return when considering flow of this a
        } else {
            stageThreeComplete = true;
        }
    }

    //once we get change and calculate the angle, then move forward to approximate the swing
    protected void stageFour() {
        System.out.println("stage 4");
        if (stageFourFristRun) {//do caluclation for angle
            deltaDistance = (jumpEncoderCount[1] - jumpEncoderCount[0]) * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
            deltaAverage = oSSPreviousAverages.get(jumpIndices[1]) - oSSPreviousAverages.get(jumpIndices[0] - 1);
            deltaHorizontalDistance = deltaAverage * ConstantsMap.DISTANCE_BETWEEN_SENSOR_CAMERAS;
            
            angleToLine = Math.acos(deltaHorizontalDistance/deltaDistance);
            desiredAngle = gyro.getAngle() + angleToLine;//This "zeros" the angle 

            swingDistance = Math.sin(angleToLine) * ConstantsMap.ROBOT_LENGTH/2;//Approximates distance the robot needs to move forward before turning to stay on line

            double previousEncoderValue = (driveSubsystem.getLeftEncoderDistance() + driveSubsystem.getRightEncoderDistance())/2;
            encoderGoal = (previousEncoderValue + swingDistance);

            stageFourFristRun = false;
        }

        //move forward to make up swing
        driveSubsystem.setRightSpeed(ConstantsMap.APPROACH_SPEED);
        driveSubsystem.setLeftSpeed(ConstantsMap.APPROACH_SPEED);

        if ((driveSubsystem.getLeftEncoderDistance() + driveSubsystem.getRightEncoderDistance())/2 > encoderGoal) {//Once we have moved enough to account for the turn
            driveSubsystem.setRightSpeed(0);
            driveSubsystem.setLeftSpeed(0);
            stageFourComplete = true;
        }
    }

    //now turn the robot to the desired angle
    protected void stageFive() {



        System.out.println("stage 5");
        if (Math.abs(desiredAngle - gyro.getAngle()) > ConstantsMap.ANGLE_TOLLERANCE) {//Checks to see if our angle of approach is too great
            if (desiredAngle - gyro.getAngle() > 0) {//These figure out which was to turn 
                driveSubsystem.setLeftSpeed((ConstantsMap.TURN_SPEED));
                driveSubsystem.setRightSpeed((ConstantsMap.TURN_SPEED * -1));
            } else {
                driveSubsystem.setLeftSpeed(( ConstantsMap.TURN_SPEED * -1));
                driveSubsystem.setRightSpeed( ConstantsMap.TURN_SPEED);
            }
        } else {
            stageFiveComplete = true;
        }
    }

    //now move to the wall, and use the old two sensor stage
    protected void stageSix() {
        System.out.println("stage 6");
        stageSixComplete = true;
    }

    //New method relying only on the sensors (a bit simpler than doing the calculations)
    protected void approach() {
        //Triggers when we have a camera on the sensor
        double frontAverage = followLineSubsystem.getLineAverage(1);

        System.out.println("LinefollowIDK start");
        System.out.println("what sensors are on?" + Arrays.toString(followLineSubsystem.getLineData(0)));
        System.out.println("what is our average?" + followLineSubsystem.getLineAverage(0));

        if (frontAverage == 0 || frontAverage == Float.NaN) {
            driveSubsystem.stop();//this sets both speeds to 0 

            System.out.println("killededed");
            return;//Kills it because the sensor is either not working or off of the tape 
        }

        if (firstRun) {
            startEncoderAvg =  (driveSubsystem.getRightEncoderDistance() + driveSubsystem.getLeftEncoderDistance()) / 2;
            encoderFinalGoal = startEncoderAvg + 16;//16 in inches 

            firstRun = false;
        }

        System.out.println("Sensor outputs: " + frontAverage);
        System.out.println("Encoder Goal: " + encoderFinalGoal + "\nCurrent Encoder Avg: " + ((driveSubsystem.getRightEncoderDistance() +              driveSubsystem.getLeftEncoderDistance()) / 2));

        if (((driveSubsystem.getRightEncoderDistance() + driveSubsystem.getLeftEncoderDistance()) / 2) < encoderFinalGoal) {
            //Doing the turning with very little error so that we never go out too far that the robot is completely off the center line 

            if (frontAverage > ConstantsMap.SENSOR_AVERAGE_CENTER + ConstantsMap.SENSOR_AVERAGE_TOLERANCE_HIGH) {//We are way to the left of the sensors, so we need to start turning (right) 
                System.out.println("we should compensate left!");
                overCompensate = true;

                driveSubsystem.tankDrive(-0.5 * ConstantsMap.APPROACH_SPEED, 2 * ConstantsMap.APPROACH_SPEED);
            } else if (frontAverage < ConstantsMap.SENSOR_AVERAGE_CENTER - ConstantsMap.SENSOR_AVERAGE_TOLERANCE_HIGH) { //Turn Left
                System.out.println("we should compensate right!");
                overCompensate = true;

                driveSubsystem.tankDrive(2 * ConstantsMap.APPROACH_SPEED, -0.5 * ConstantsMap.APPROACH_SPEED);
            } else if (Math.abs(ConstantsMap.SENSOR_AVERAGE_CENTER - frontAverage) <= ConstantsMap.SENSOR_AVERAGE_TOLERANCE_LOW) {//We are within the tolerance, so we just move forward 
                System.out.println("we have fixed it, so no more overcompensating!");
                overCompensate = false;

                driveSubsystem.tankDrive(ConstantsMap.APPROACH_SPEED, ConstantsMap.APPROACH_SPEED);
            } else if(frontAverage < ConstantsMap.SENSOR_AVERAGE_CENTER + ConstantsMap.SENSOR_AVERAGE_TOLERANCE_HIGH 
                && frontAverage > ConstantsMap.SENSOR_AVERAGE_CENTER + ConstantsMap.SENSOR_AVERAGE_TOLERANCE_LOW
                && overCompensate) {
                System.out.println("compensating left!");

                driveSubsystem.tankDrive(-0.5 * ConstantsMap.APPROACH_SPEED, 2 * ConstantsMap.APPROACH_SPEED);
            }
            else if(frontAverage > ConstantsMap.SENSOR_AVERAGE_CENTER - ConstantsMap.SENSOR_AVERAGE_TOLERANCE_HIGH 
                && frontAverage < ConstantsMap.SENSOR_AVERAGE_CENTER - ConstantsMap.SENSOR_AVERAGE_TOLERANCE_LOW
                && overCompensate){
                System.out.println("compensating right!");

                driveSubsystem.tankDrive(2 * ConstantsMap.APPROACH_SPEED, -0.5 * ConstantsMap.APPROACH_SPEED);
            }
        } else {
<<<<<<< HEAD
            driveSubsystem.setLeftSpeed(0);
            driveSubsystem.setRightSpeed(0);
=======
            driveSubsystem.stop();

            runningFLC = false;//We have completed the process 
>>>>>>> f8e92135e9676795fff5d4989b69e53ca011de39
            return;
        }

        System.out.println("LinefollowIDK end");
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return stageSixComplete;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("FollowLineCommand end");
    }

    // Called for manual interruption of command
    protected void kill() {
        stageOneComplete = true;
        stageTwoComplete = true;
        stageThreeComplete = true; 
        stageFourComplete = true;
        stageFiveComplete = true;
        stageSixComplete = true;

        System.out.println("FollowLineCommand kill");
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("FollowLineCommand interrupted");
        kill();
    }
}