package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.pids.EncoderMotorPID;
import frc.robot.TalonSet;
import frc.robot.ConstantsMap;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    ArmSubsystem armSubsystem = Robot.armSubsystem;
    EncoderMotorPID wristLevelPID, armMovementPID;
    private boolean enableLevelWrist;

    public ArmCommand() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        //Generalizes the interface between PID for each mechanism and its components.
        wristLevelPID = new EncoderMotorPID(armSubsystem.getWristEncoder(), armSubsystem.getWristTalons(), ConstantsMap.WRIST_ZERO_KP,
            ConstantsMap.WRIST_ZERO_KI, ConstantsMap.WRIST_ZERO_KD, ConstantsMap.WRIST_ZERO_KF).setOutputRange(-1,1);
        armMovementPID = new EncoderMotorPID(armSubsystem.getShoulderEncoder(), armSubsystem.getWristTalons(), ConstantsMap.SHOULDER_MV_KP,
            ConstantsMap.SHOULDER_MV_KI, ConstantsMap.SHOULDER_MV_KD, ConstantsMap.SHOULDER_MV_KF).setOutputRange(-1,1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double amountToMoveShoulderJoint = XboxMap.controlShoulderJoint();
        double amountToMoveWristJoint = XboxMap.controlWristJoint();
        
        if(Math.abs(amountToMoveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY && Math.abs(amountToMoveShoulderJoint) < ConstantsMap.JOYSTICK_SENSITIVITY) {
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
            wristLevelPID.enable();
            armMovementPID.enable();
        }

        //If the joystick is at 0 and the button was true previously then run levelling of the wrist
        //or the current button is not equal to the previous press when the joystick is at 0
        //Otherwise the wrist should not be levelling at the moment
        //Run so long as we are not moving the joystick, and the switch is on and or the enable level wrist button is hit 
        //It is only turned off if you move the joystick 
        if((Math.abs(amountToMoveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY) && (enableLevelWrist || XboxMap.enableWristLevelling())) {
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
        if (!enableLevelWrist && Math.abs(amountToMoveWristJoint) < ConstantsMap.JOYSTICK_SENSITIVITY
                && Math.abs(getRelativeLevelledAngle(armSubsystem.getWristDistance() - wristLevelPID.getSetpoint())) > ConstantsMap.WRIST_TOLERANCE) {
            wristLevelPID.setSetpoint(getRelativeLevelledAngle(armSubsystem.getWristDistance()));
        } else if (Math.abs(amountToMoveWristJoint) > ConstantsMap.JOYSTICK_SENSITIVITY) {
            wristLevelPID.disable();
            armSubsystem.setWristJointSpeed(amountToMoveWristJoint * ConstantsMap.WRIST_SPEED_MULT);
        }

        //If the shoulder is not being moved it will maintain position
        //then the angle of the shoulder will be preserved
        //Otherwise the shoulder is able to move freely without PID
        //The PID will continue working until it has reached its target or the shoulder joystick moves
        if (Math.abs(amountToMoveShoulderJoint) < ConstantsMap.JOYSTICK_SENSITIVITY
                && Math.abs(getShoulderEncoderAngle() - armMovementPID.getSetpoint()) > ConstantsMap.SHOULDER_TOLERANCE) {
            armMovementPID.setSetpoint(armSubsystem.getShoulderDistance() / ConstantsMap.SHOULDER_GEAR_RATIO + ConstantsMap.SHOULDER_OFFSET);
            armMovementPID.enable();
        } else if (Math.abs(amountToMoveShoulderJoint) > ConstantsMap.JOYSTICK_SENSITIVITY) {
            armMovementPID.disable();
            armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint * ConstantsMap.SHOULDER_SPEED_MULT);        
        }
        
        //Check to see if the wrist or shoulder has reached the limits and we need to stop them 
        if (armSubsystem.getShoulderLowerLimit()) {
            if (armSubsystem.getShoulderSpeed() < 0) {
                armSubsystem.setShoulderJointSpeed(0);
            }
        }

        if (armSubsystem.getShoulderUpperLimit()) {
            if (armSubsystem.getShoulderSpeed() > 0) {
                armSubsystem.setShoulderJointSpeed(0);
            }
        }

        if (armSubsystem.getWristLowerLimit()) {
            if (armSubsystem.getWristSpeed() < 0) {
                armSubsystem.setShoulderJointSpeed(0);
            }
        }

        if (armSubsystem.getWristUpperLimit()) {
            if (armSubsystem.getWristSpeed() > 0) {
                armSubsystem.setShoulderJointSpeed(0);
            }
        }
    }

    /**
     * Computes the angle of the shoulder as a ratio of the motors to the arm. 
     * 0 degrees is straight down and positive is infront of the robot.
     * @return angle in degrees
     */
    private double getShoulderEncoderAngle() {
        return ConstantsMap.SHOULDER_OFFSET + armSubsystem.getShoulderDistance() / ConstantsMap.SHOULDER_GEAR_RATIO;
    }

    /**
     * Gets the angle relative to the encoder on the wrist that the wrist should go to be level.
     * @return
     */
    private double getWristLevelledAngle() {
        return getShoulderEncoderAngle() + ConstantsMap.WRIST_OFFSET;
    }

    /**
     * Computes the Encoder angle needed to maintain a constant world space angle
     * @param angle of the current encoder reading that would like to be maintained in world space
     * @return angle that the encoder should go to maintain in world space as the shoulder moves
     */
    private double getRelativeLevelledAngle(double angle) {
        return angle - getWristLevelledAngle() + getShoulderEncoderAngle();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}