package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.pids.EncoderMotorPID;
import frc.robot.TalonSet;
import frc.robot.ConstantsMap;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    ArmSubsystem armSubsystem = Robot.armSubsystem;
    EncoderMotorPID wristLevelPID, armMovementPID;
    private boolean enableLevelWrist = true;
    boolean manualMode = true;
    private double armAngle;
    private double wristAngle;
    public ArmCommand() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Arm Command Started");

        //Generalizes the interface between PID for each mechanism and its components.
        wristLevelPID = new EncoderMotorPID(armSubsystem.getWristTalons(), ConstantsMap.WRIST_ZERO_KP,
            ConstantsMap.WRIST_ZERO_KI, ConstantsMap.WRIST_ZERO_KD, ConstantsMap.WRIST_ZERO_KF).setOutputRange(-1,1);
        armMovementPID = new EncoderMotorPID(armSubsystem.getShoulderTalons(), ConstantsMap.SHOULDER_MV_KP,
            ConstantsMap.SHOULDER_MV_KI, ConstantsMap.SHOULDER_MV_KD, ConstantsMap.SHOULDER_MV_KF).setOutputRange(-1,1);
        
        armAngle = armSubsystem.getShoulderEncoderAngle();
        wristAngle = armSubsystem.getWristEncoderAngle();
        wristLevelPID.getPID().setInputRange(-360, 360);
        wristLevelPID.getPID().setOutputRange(-1, 1);
        wristLevelPID.getPID().setContinuous();

        armMovementPID.getPID().setInputRange(-360, 360);
        armMovementPID.getPID().setOutputRange(-1, 1);

        armMovementPID.getPID().setContinuous();
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        SmartDashboard.putBoolean("Mnaual Mode", manualMode);

        if(XboxMap.zeroWrist()){
            armSubsystem.zeroWrist();
        }
        else if(XboxMap.zeroShoulder()){
            armSubsystem.zeroShoulder();
        }
        else if(XboxMap.toggleArmControl()){
            if(manualMode){
                //wristLevelPID.enable();
                armMovementPID.enable();
            } 
            else{
                //wristLevelPID.disable();
                armMovementPID.disable();
            }
            manualMode = !manualMode;
        }
        double amountToMoveShoulderJoint = XboxMap.controlShoulderJoint();
        double amountToMoveWristJoint = XboxMap.controlWristJoint();
        if (Math.abs(amountToMoveShoulderJoint) < 0.1) {
            amountToMoveShoulderJoint = 0;
        }
        if (Math.abs(amountToMoveWristJoint) < 0.1) {
            amountToMoveWristJoint = 0;
        }
        
        if(manualMode){
            armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint);
            armSubsystem.setWristJointSpeed(amountToMoveWristJoint);
        }
        else{

        
            if (armSubsystem.getWristLowerLimit()) {
                if (amountToMoveWristJoint < 0) {
                    
                    armSubsystem.setWristJointSpeed(0);
                    amountToMoveWristJoint = 0;
                    System.out.println("Yeeting");
                }
            }
            else{
                armAngle += amountToMoveShoulderJoint;
                wristAngle += amountToMoveWristJoint;
            }
            if(XboxMap.enableWristLevelling()){
                enableLevelWrist = !enableLevelWrist;
            }

            if(armAngle > ConstantsMap.SHOULDER_MAX_ANGLE){
                armAngle = ConstantsMap.SHOULDER_MAX_ANGLE;
            }
            if(armAngle < ConstantsMap.SHOULDER_MIN_ANGLE){
                armAngle = ConstantsMap.SHOULDER_MIN_ANGLE;
            }
            if(wristAngle > ConstantsMap.WRIST_MAX_ANGLE){
                armAngle = ConstantsMap.WRIST_MAX_ANGLE;
            }
            if(wristAngle < ConstantsMap.WRIST_MIN_ANGLE){
                armAngle = ConstantsMap.WRIST_MIN_ANGLE;
            }
        }
        //armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint);
        //armSubsystem.setWristJointSpeed(amountToMoveWristJoint);
        
        /* if(XboxMap.enableCargoPreset()){
            wristLevelPID.enable();
            wristLevelPID.setSetpoint(45);
        }
        else
            wristLevelPID.disable();
        */

        
            //Do not press multiple presets at the same time.
            //TODO Set the angles that the arm and wrist should go to.
            if(XboxMap.enableCargoPreset()) {
                wristLevelPID.setSetpoint(getRelativeLevelledAngle(0));
                armMovementPID.setSetpoint(getRelativeLevelledAngle(0));
            }

            else if(XboxMap.enableLevel2Preset()) {
                wristLevelPID.setSetpoint(getRelativeLevelledAngle(0));
                armMovementPID.setSetpoint(getRelativeLevelledAngle(0));
            }

            else if(XboxMap.enableLevel3Preset()) {
                wristLevelPID.setSetpoint(getRelativeLevelledAngle(0));
                armMovementPID.setSetpoint(getRelativeLevelledAngle(0));
            }
            else if(enableLevelWrist){
                wristLevelPID.setSetpoint(getRelativeLevelledAngle(armAngle));
                armMovementPID.setSetpoint(armAngle);
            }
            else{
                wristLevelPID.setSetpoint(wristAngle);
                armMovementPID.setSetpoint(armAngle);
            }
        

        
       /*  //If the joystick is at 0 and the button was true previously then run levelling of the wrist
        //or the current button is not equal to the previous press when the joystick is at 0
        //Otherwise the wrist should not be levelling at the moment
        //Run so long as we are not moving the joystick, and the switch is on and or the enable level wrist button is hit 
        //It is only turned off if you move the joystick 

        System.out.println("Wassup");
        if((Math.abs(amountToMoveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY) && (enableLevelWrist || )) {
            wristLevelPID.setSetpoint(getWristLevelledAngle());
            wristLevelPID.enable();
            enableLevelWrist = true;
        } else {
            enableLevelWrist = false;
        }

        //If the wrist is not being moved and not being levelled
        //then the angle of the wrist will be preserved
        //Otherwise the wrist is able to move freely without PID for both types of levelling
        //The PID will continue working until it has reached its target, the shoulder joystick moves, or enableLevelWrist is on
        System.out.println("Wassup again");
        if (!enableLevelWrist && Math.abs(amountToMoveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY
                && Math.abs(getRelativeLevelledAngle(armSubsystem.getWristEncoderAngle() - wristLevelPID.getSetpoint())) > ConstantsMap.WRIST_TOLERANCE) {
            wristLevelPID.setSetpoint(getRelativeLevelledAngle(armSubsystem.getWristEncoderAngle()));
        } else if (Math.abs(amountToMoveWristJoint) > ConstantsMap.JOYSTICK_SENSITIVITY) {
            wristLevelPID.disable();
            armSubsystem.setWristJointSpeed(amountToMoveWristJoint * ConstantsMap.WRIST_SPEED_MULT);
        }

        //If the shoulder is not being moved it will maintain position
        //then the angle of the shoulder will be preserved
        //Otherwise the shoulder is able to move freely without PID
        //The PID will continue working until it has reached its target or the shoulder joystick moves
        System.out.println("Wassup again again");
        if (Math.abs(amountToMoveShoulderJoint) < ConstantsMap.JOYSTICK_SENSITIVITY
                && Math.abs(getShoulderAngle() - armMovementPID.getSetpoint()) > ConstantsMap.SHOULDER_TOLERANCE) {
            armMovementPID.setSetpoint(armSubsystem.getShoulderEncoderAngle() / ConstantsMap.SHOULDER_GEAR_RATIO + ConstantsMap.SHOULDER_OFFSET);
            armMovementPID.enable();
        } else if (Math.abs(amountToMoveShoulderJoint) > ConstantsMap.JOYSTICK_SENSITIVITY) {
            armMovementPID.disable();
            armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint * ConstantsMap.SHOULDER_SPEED_MULT);        
        }
         */
        
        //Check to see if the wrist or shoulder has reached the limits and we need to stop them 
        // System.out.println("Hello");
        // if (armSubsystem.getShoulderLowerLimit()) {
        //     if (armSubsystem.getShoulderSpeed() < 0) {
        //         armSubsystem.setShoulderJointSpeed(0);
        //     }
        // }

        // if (armSubsystem.getShoulderUpperLimit()) {
        //     if (armSubsystem.getShoulderSpeed() > 0) {
        //         armSubsystem.setShoulderJointSpeed(0);
        //     }
        // }

        

        // if (armSubsystem.getWristUpperLimit()) {
        //     if (armSubsystem.getWristSpeed() > 0) {
        //         armSubsystem.setShoulderJointSpeed(0);
        //     }
        // }

    }

    /**
     * Computes the angle of the shoulder as a ratio of the motors to the arm. 
     * 0 degrees is straight down and positive is infront of the robot.
     * @return angle in degrees
     */
    private double getShoulderAngle() {
        return ConstantsMap.SHOULDER_OFFSET + armSubsystem.getShoulderEncoderAngle() / ConstantsMap.SHOULDER_GEAR_RATIO;
    }

    /**
     * Gets the angle relative to the encoder on the wrist that the wrist should go to be level.
     * @return
     */
    private double getWristLevelledAngle() {
        return armSubsystem.getShoulderEncoderAngle();//Do the math here right now 
    }

    /**
     * Computes the Encoder angle needed to maintain a constant world space angle
     * @param angle of the current encoder reading that would like to be maintained in world space
     * @return angle that the encoder should go to maintain in world space as the shoulder moves
     */
    private double getRelativeLevelledAngle(double angle) {
        return -1 * getShoulderAngle();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("Arm Command Stopped");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}