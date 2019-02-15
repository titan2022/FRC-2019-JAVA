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
import frc.robot.subsystems.ArmSubsystem2;

public class ArmCommand2 extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    EncoderMotorPID wristLevelPID, armMovementPID;
    private boolean enableLevelWrist = true;
    boolean manualMode = true;
    private double shoulderAngle;
    private double wristAngle;
    private double actualShoulderAngle;
    private double actualWristAngle;
    public ArmCommand2() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Arm Command Started");

        
        shoulderAngle = armSubsystem.getShoulderEncoderAngle();
        wristAngle = armSubsystem.getWristEncoderAngle();
        manualMode = true;
        enableLevelWrist = false;
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        SmartDashboard.putBoolean("Manual Mode", manualMode);
        SmartDashboard.putBoolean("Level Mode", enableLevelWrist);

        actualShoulderAngle = armSubsystem.getShoulderEncoderAngle();
        actualWristAngle = armSubsystem.getWristEncoderAngle();

        if(XboxMap.zeroWrist()){
            armSubsystem.zeroWrist();
        }
        else if(XboxMap.zeroShoulder()){
            armSubsystem.zeroShoulder();
        }
        else if(XboxMap.toggleArmControl()){            
            if(manualMode){
                shoulderAngle = armSubsystem.getShoulderEncoderAngle();
                wristAngle = armSubsystem.getWristEncoderAngle();

            }
            manualMode = !manualMode;

        }
        if(XboxMap.enableWristLevelling()){
            enableLevelWrist = !enableLevelWrist;
        }
        double amountToMoveShoulderJoint = XboxMap.controlShoulderJoint();
        double amountToMoveWristJoint = XboxMap.controlWristJoint();
        
        if (Math.abs(amountToMoveShoulderJoint) < 0.1) {
            amountToMoveShoulderJoint = 0;
        }
        if (Math.abs(amountToMoveWristJoint) < 0.1) {
            amountToMoveWristJoint = 0;
        }
        amountToMoveShoulderJoint *= ConstantsMap.SHOULDER_CHANGE_SETPOINT_SPEED;
        amountToMoveWristJoint *= ConstantsMap.WRIST_CHANGE_SETPOINT_SPEED;

        if(manualMode){
            armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint);
            armSubsystem.setWristJointSpeed(amountToMoveWristJoint);
        }
        else{                 
            if(Math.abs((shoulderAngle + amountToMoveShoulderJoint)-actualShoulderAngle)<ConstantsMap.SHOULDER_FOLLOW_DISTANCE){
                shoulderAngle += amountToMoveShoulderJoint;
            } 
            if(Math.abs((wristAngle + amountToMoveWristJoint)-actualWristAngle)<ConstantsMap.WRIST_FOLLOW_DISTANCE){
                wristAngle += amountToMoveWristJoint;
            } 
            if(shoulderAngle > ConstantsMap.SHOULDER_MAX_ANGLE){
                shoulderAngle = ConstantsMap.SHOULDER_MAX_ANGLE;
            }
            if(shoulderAngle < ConstantsMap.SHOULDER_MIN_ANGLE){
                shoulderAngle = ConstantsMap.SHOULDER_MIN_ANGLE;
            }
            if(wristAngle > ConstantsMap.WRIST_MAX_ANGLE){
                wristAngle = ConstantsMap.WRIST_MAX_ANGLE;
            }
            if(wristAngle < ConstantsMap.WRIST_MIN_ANGLE){
                wristAngle = ConstantsMap.WRIST_MIN_ANGLE;
            }
            
            armSubsystem.setShoulderSetPoint(shoulderAngle);
            if(enableLevelWrist){
                armSubsystem.setWristSetPoint(-shoulderAngle);
            }
            else{
                armSubsystem.setWristSetPoint(wristAngle);

            }
        }
        SmartDashboard.putNumber("Shoulder setPoint", shoulderAngle);
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