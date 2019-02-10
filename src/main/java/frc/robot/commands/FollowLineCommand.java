package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    

    //Checks to see if we are running rn 
    protected boolean runningFLC;

    //Maybe this will work variables 
    protected double startEncoderAvg;
    protected double encoderFinalGoal;

    protected boolean firstRun;
    //Other stuff that we need 
    

    //moar variables
    private boolean overCompensate = false;

    protected double estimatedDistanceToWall;

    private double lineAngle;
    private double lineDistance;
    private boolean lineRightSide;
    public FollowLineCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
    }
    PIDController drive;
    PIDController turn;
    PIDOutput output = new PIDOutput(){
    
        @Override
        public void pidWrite(double output) {
            
        }
    };
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        driveSubsystem.resetGyro();
        System.out.println("FollowLineCommand init");
        driveSubsystem.enableBrake();
        driveSubsystem.resetEncoders();
        driveSubsystem.resetGyro();
        lineAngle = SmartDashboard.getNumber("LineStartAngle", 0.0);
        lineDistance = SmartDashboard.getNumber("LineStartDistance", 0.0);
        //lineRightSide  = SmartDashboard.getBoolean("LineRightSide", true);
        PIDSource source = new PIDSource(){
        
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                
                
            }
        
            @Override
            public double pidGet() {
                return driveSubsystem.getLeftEncoderDistance();
            }
        
            @Override
            public PIDSourceType getPIDSourceType() {
                return null;
            }
        };
        
        drive = new PIDController(1, 0, 0, source, output);
        turn = new PIDController(1, 0, 0, driveSubsystem.getGyro(), output);
        drive.setInputRange(-180, 180);        
        drive.setAbsoluteTolerance(ConstantsMap.PID_PERCENT_TOLERANCE);
        drive.setSetpoint(lineDistance);
        System.out.println("Set drive setpoint: " + lineDistance);
        drive.setOutputRange(-ConstantsMap.PID_OUTPUT_MAX, ConstantsMap.PID_OUTPUT_MAX);

        turn.setInputRange(-180, 180);
        turn.setAbsoluteTolerance(ConstantsMap.PID_PERCENT_TOLERANCE);        
        turn.setSetpoint(90-lineAngle);
        turn.setOutputRange(-.5, .5);
        drive.enable();
        

    }

    //this is to be called upon initialization and whenever the button is hit twice
    protected void setupForRun() { 
        //turns on runningFLC 
        System.out.println("we made it true");
        runningFLC = true;
        
        //setup stage 1 variables
        stageOneComplete = false;
        
        //maybe 
        firstRun = true;
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(drive.isEnabled()){
            if(drive.onTarget()){
                drive.setEnabled(false);
                turn.setEnabled(true);
            }
            else{
                double output = drive.get();
                System.out.println(output);

                driveSubsystem.tankDrive(output, output);
            }
        }
        else if(turn.isEnabled()){
            if(turn.onTarget()){
                end();
            }
            else{
                double output = turn.get();
                driveSubsystem.tankDrive(-output, output);
            }
        }
                
    }
  
    
    //New method relying only on the sensors (a bit simpler than doing the calculations)
    protected void approach() {
        //Triggers when we have a camera on the sensor
        System.out.println("fdjskal;fjkdsajfkl;dsajfkld;sajfkl;dsajkfldsjakl;fjdsakl;fjdklsajflkd;sa");
        double frontAverage = followLineSubsystem.getLineAverage(1);

        System.out.println("approach begin");

        System.out.println("raw sensor data:" + Arrays.toString(followLineSubsystem.getRawData()));
        System.out.println("t/f sensor data:" + Arrays.toString(followLineSubsystem.getLineData(0)));

        if (frontAverage == 0 || frontAverage == Float.NaN) {
            driveSubsystem.stop();//this sets both speeds to 0 

            System.out.println("killededed");
            return;//Kills it because the sensor is either not working or off of the tape 
        }

        if (firstRun) {//this sets up the first few values we need 
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
            driveSubsystem.stop();
            System.out.println("We are bad");
            runningFLC = false;//We have completed the process 
            return;
        }

        System.out.println("approach end");
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        System.out.println("we are really dumb " + runningFLC);
        return !runningFLC;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("FollowLineCommand end");
    }

    // Called for manual interruption of command
    protected void kill() {
        runningFLC = false;

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




















/*



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


public class FollowLineCommand extends Command {
    FollowLineSubsystem followLineSubsystem = Robot.followLineSubsystem;
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;

    //stage one variables (Vision Stage)
    protected boolean stageOneComplete;

    /*
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
    

    //Checks to see if we are running rn 
    protected boolean runningFLC;

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
        //turns on runningFLC 
        System.out.println("we made it true");
        runningFLC = true;
        
        //setup stage 1 variables
        stageOneComplete = false;
        /*
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
            setupForRun();//reset for next run
            somehow mark that we are done, maybe interrrupt?
        }else{
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
        System.out.println("We execute");
        approach();
    }
    
    /*DELETE THIS AT SOME POINT LATER WHEN OTHER METHODS WORK 
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
        System.out.println("fdjskal;fjkdsajfkl;dsajfkld;sajfkl;dsajkfldsjakl;fjdsakl;fjdklsajflkd;sa");
        double frontAverage = followLineSubsystem.getLineAverage(1);

        System.out.println("approach begin");

        System.out.println("raw sensor data:" + Arrays.toString(followLineSubsystem.getRawData()));
        System.out.println("t/f sensor data:" + Arrays.toString(followLineSubsystem.getLineData(0)));

        if (frontAverage == 0 || frontAverage == Float.NaN) {
            driveSubsystem.stop();//this sets both speeds to 0 

            System.out.println("killededed");
            return;//Kills it because the sensor is either not working or off of the tape 
        }

        if (firstRun) {//this sets up the first few values we need 
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
            driveSubsystem.stop();
            System.out.println("We are bad");
            runningFLC = false;//We have completed the process 
            return;
        }

        System.out.println("approach end");
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        System.out.println("we are really dumb " + runningFLC);
        return !runningFLC;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("FollowLineCommand end");
    }

    // Called for manual interruption of command
    protected void kill() {
        runningFLC = false;

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
*/