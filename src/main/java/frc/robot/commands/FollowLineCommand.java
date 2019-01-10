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

    boolean[] previousValues;
    ArrayList<Double> oSSPreviousAverages;
    int numOfJumps; 
    int[] jumpIndices; // these will be the index after the change
    double[] jumpEncoderDistance; // relative to the start of stage 2
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

    //Other stuff that we need 
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
        jumpEncoderDistance = new double[]{0.0, 0.0};
        startTime = System.nanoTime();
        previousValues = new boolean[]{false, false, false, false, false, false, false, false};

        //stage 4
        stageFourComplete = false;
        stageFourFristRun = true;
        
        //stage 5
        stageFiveComplete = false;

        //stage 6
        stageSixComplete = false;    
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        

        /*if(limit switch hit){
            setupForRun();//reset for next run
            somehow mark that we are done, maybe interrrupt?
        }else{*/
        if(!stageOneComplete) {
            stageOne();
        } else if(!stageTwoComplete) {
            stageTwo();
        } else if(!stageThreeComplete) {
            stageThree();
        } else if(!stageFourComplete) {
            stageFour();
        } else if(!stageFiveComplete) {
            stageFive();
        } else if(!stageSixComplete) {
            stageSix();
        } 
    }

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

        driveSubsystem.setRightSpeed(ConstantsMap.STAGE_TWO_SPEED);
        driveSubsystem.setLeftSpeed(ConstantsMap.STAGE_TWO_SPEED);

        if ((driveSubsystem.getLeftEncoderDistance() > leftEncoderDistanceGoal)||(driveSubsystem.getRightEncoderDistance() > rightEncoderDistanceGoal)) {
            stageTwoComplete = true;
        }
    }

    //once we are over the line, then watch for the horizontal change
    protected void stageThree() {
        System.out.println("stage 3");
        boolean[] isFrontCameraOnStrip = new boolean[8];
        
        int[] frontCameraValues = followLineSubsystem.getRawData();

        for(int i = 0; i < 8; i++){
            if(frontCameraValues[i] < ConstantsMap.LOW_CUTOFF){
                isFrontCameraOnStrip[i] = false;
            } else if(frontCameraValues[i] > ConstantsMap.HIGH_CUTOFF){
                isFrontCameraOnStrip[i] = true;
            } else{
                isFrontCameraOnStrip[i] = previousValues[i];
            }
        }

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

        System.out.println(Arrays.toString(isFrontCameraOnStrip));
        System.out.println(followLineSubsystem.getData());

        //this gets the jumps in average
        if(numOfJumps < 2) {
            double frontAverage = followLineSubsystem.getLineAverage(1);
            oSSPreviousAverages.add(frontAverage);
            int oSSSize = oSSPreviousAverages.size();

            if(oSSSize > 1 && numOfJumps < 2) {//just making sure we dont step outside oSSpreviousaverages
                if(Math.abs(oSSPreviousAverages.get(oSSSize - 1) - oSSPreviousAverages.get(oSSSize - 2)) > 0.25){
                    //now we have a step, so we can set our values

                    jumpIndices[numOfJumps] = oSSSize - 1;
                    jumpEncoderDistance[numOfJumps] = (driveSubsystem.getLeftEncoderDistance() + driveSubsystem.getRightEncoderDistance()) / 2; 
                    
                    
                    System.out.println("" + numOfJumps + ", " + jumpIndices[numOfJumps] + ", " + jumpEncoderDistance[numOfJumps]);

                    numOfJumps++;
                }
            }
            return; // please note this return when considering flow of this a
        } else {
            stageThreeComplete = true;
        }

        previousValues = isFrontCameraOnStrip;
    }

    //once we get change and calculate the angle, then move forward to approximate the swing
    protected void stageFour() {
        System.out.println("stage 4");
        if (stageFourFristRun) {//do caluclation for angle
            deltaDistance = jumpEncoderDistance[1] - jumpEncoderDistance[0];
            deltaAverage = oSSPreviousAverages.get(jumpIndices[1]) - oSSPreviousAverages.get(jumpIndices[0] - 1);
            deltaHorizontalDistance = deltaAverage * ConstantsMap.DISTANCE_BETWEEN_SENSOR_CAMERAS;
            
            angleToLine = Math.toDegrees(Math.acos(deltaHorizontalDistance/deltaDistance));
            desiredAngle = (gyro.getAngle() + angleToLine);//This "zeros" the angle 

            swingDistance = Math.sin(Math.toRadians(angleToLine)) * ConstantsMap.ROBOT_LENGTH/2;//Approximates distance the robot needs to move forward before turning to stay on line

            double previousEncoderValue = (driveSubsystem.getLeftEncoderDistance() + driveSubsystem.getRightEncoderDistance())/2;
            encoderGoal = (previousEncoderValue + swingDistance);

            stageFourFristRun = false;

            System.out.println("deltaDistance:" + deltaDistance);
            System.out.println("deltaAverage:" + deltaAverage);
            System.out.println("deltaHorizontalDistance:" + deltaHorizontalDistance);
            System.out.println("angleToline:" + angleToLine);
            System.out.println("currentAngle:" + gyro.getAngle());
            System.out.println("desiredAngle:" + desiredAngle);
            System.out.println("swingDistance:" + swingDistance);
            System.out.println("previousEncoderValue:" + previousEncoderValue);
            System.out.println("encoderGoal:" + encoderGoal);
        }

        stageFourComplete = true;

        //move forward to make up swing
        driveSubsystem.setRightSpeed(ConstantsMap.STAGE_TWO_SPEED);
        driveSubsystem.setLeftSpeed(ConstantsMap.STAGE_TWO_SPEED);

        if ((driveSubsystem.getLeftEncoderDistance() + driveSubsystem.getRightEncoderDistance())/2 > encoderGoal) {//Once we have moved enough to account for the turn
            stageFourComplete = true;
        }
    }

    //now turn the robot to the desired angle
    protected void stageFive() {
        System.out.println("stage 5");
        if (Math.abs(desiredAngle - gyro.getAngle()) > ConstantsMap.ANGLE_TOLLERANCE) {//Checks to see if our angle of approach is too great
            if (gyro.getAngle() - desiredAngle > 0) {//These figure out which was to turn 
                //turn left
                System.out.println("turnLeft");
                driveSubsystem.setLeftSpeed(ConstantsMap.TURN_SPEED * -1);
                driveSubsystem.setRightSpeed(ConstantsMap.TURN_SPEED * 1);
            } else {
                //turn right
                System.out.println("turnRight");
                driveSubsystem.setLeftSpeed(ConstantsMap.TURN_SPEED * 1);
                driveSubsystem.setRightSpeed(ConstantsMap.TURN_SPEED * -1);
            }
        } else {
            stageFiveComplete = true;
            driveSubsystem.setRightSpeed(0);
            driveSubsystem.setLeftSpeed(0);
            System.out.println("fuck");
        }
    }

    //now move to the wall, and use the old two sensor stage
    protected void stageSix() {
        System.out.println("stage 6");
        stageSixComplete = true;
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