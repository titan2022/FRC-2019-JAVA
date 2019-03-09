package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsMap;
import frc.robot.ControlPanelMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;

public class ArmCommand2 extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    private boolean enableLevelWrist = true;
    boolean manualMode = true;
    private double shoulderAngle;
    private double wristAngle;
    private double actualShoulderAngle;
    private double actualWristAngle;
    private double oldShoulderAngle;
    private double oldWristAngle;
    
    public ArmCommand2() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Arm Command Started");

        
        shoulderAngle = armSubsystem.getShoulderEncoderAngle();
        wristAngle = armSubsystem.getWristEncoderAngle();
        manualMode = false;
        enableLevelWrist = false;
        oldShoulderAngle = 0;
        oldWristAngle = 0;
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        SmartDashboard.putBoolean("Manual Mode", manualMode);
        SmartDashboard.putBoolean("Level Mode", enableLevelWrist);
 
        /* if(ControlPanelMap.toggleArmManualControl()){            
            if(manualMode){
                shoulderAngle = armSubsystem.getShoulderEncoderAngle();
                wristAngle = armSubsystem.getWristEncoderAngle();

            }
            manualMode = !manualMode;

        }
        if(ControlPanelMap.enableWristLevelling()){
            enableLevelWrist = !enableLevelWrist;
        } */
        double amountToMoveShoulderJoint = ControlPanelMap.controlShoulderJoint();
        double amountToMoveWristJoint = ControlPanelMap.controlWristJoint();
        
        if (Math.abs(amountToMoveShoulderJoint) < 0.1) {
            amountToMoveShoulderJoint = 0;
        }
        if (Math.abs(amountToMoveWristJoint) < 0.1) {
            amountToMoveWristJoint = 0;
        }
        
       /*  amountToMoveShoulderJoint *= ConstantsMap.SHOULDER_CHANGE_SETPOINT_SPEED;
        amountToMoveWristJoint *= ConstantsMap.WRIST_CHANGE_SETPOINT_SPEED; */

        actualShoulderAngle = armSubsystem.getShoulderEncoderAngle();
        actualWristAngle = armSubsystem.getWristEncoderAngle();
        /* if(SmartDashboard.getBoolean("Debug Mode", false)){
            armSubsystem.setShoulderJointSpeed(.1 * amountToMoveShoulderJoint);
            armSubsystem.setWristJointSpeed(.1 * amountToMoveWristJoint);
        } */
        if(armSubsystem.getManualMode()){
            armSubsystem.setShoulderJointSpeed(amountToMoveShoulderJoint/3);
            armSubsystem.setWristJointSpeed(amountToMoveWristJoint);
        }
        else{                 
            if(Math.abs((shoulderAngle + amountToMoveShoulderJoint)-actualShoulderAngle)<ConstantsMap.SHOULDER_FOLLOW_DISTANCE){
                shoulderAngle += amountToMoveShoulderJoint;
            } 
            if(Math.abs((wristAngle + amountToMoveWristJoint)-actualWristAngle)<ConstantsMap.WRIST_FOLLOW_DISTANCE){
                wristAngle += amountToMoveWristJoint;
            } 
        
            if(shoulderAngle != oldShoulderAngle){
                armSubsystem.setShoulderSetPoint(shoulderAngle);

            }
            if(wristAngle != oldWristAngle || armSubsystem.getLevelMode()){
                if(armSubsystem.getLevelMode()){
                    armSubsystem.setWristSetPoint(-shoulderAngle);
                }
                else{
                    armSubsystem.setWristSetPoint(wristAngle);
    
                }
            }
            oldWristAngle = wristAngle;
            oldShoulderAngle = shoulderAngle;
            
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
        System.out.println("Arm Command Intrupted");

        armSubsystem.setShoulderJointSpeed(0);
        armSubsystem.setWristJointSpeed(0);;

    }
}