package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.ControlPanelMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;

public class ArmPresetCommand extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    double sPreset = 1000;
    double wPreset = 1000;
    boolean shoulderFinished;
    boolean levelMode;
    boolean initialized = false;
    boolean shoulderDone = false;
    double startWrist;
    public ArmPresetCommand(double sAngle,double wAngle) {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);
        sPreset = sAngle;
        wPreset = wAngle;
        levelMode = false;
        //startWrist = (sAngle - armSubsystem.getShoulderEncoderAngle())/2;

    }
    public ArmPresetCommand(double sAngle) {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);
        sPreset = sAngle;
        levelMode = true;

    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        
        System.out.println("Start Preset: " + sPreset + " Wrist: " + wPreset);
        armSubsystem.setShoulderSetPoint(sPreset);
        //armSubsystem.setWristSetPoint(0);
        initialized = true;
        shoulderFinished = false;
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(armSubsystem.getShoulderEncoderAngle() < (ConstantsMap.SHOULDER_MIN_ANGLE + 5)) {
            armSubsystem.setWristSetPoint(ConstantsMap.WRIST_MAX_ANGLE);
        }       
        else if(levelMode){
            armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle()); 
        }
        else if(!armSubsystem.isShoulderAtSetPoint(sPreset) && !shoulderFinished){
            armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle()); 
        } 
        else{        
                if(!shoulderFinished)    armSubsystem.setWristSetPoint(wPreset);  
                shoulderFinished = true;
          
        }
        //armSubsyste

    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        //return false;
        if(Math.abs(ControlPanelMap.controlShoulderJoint()) > .1 || Math.abs(ControlPanelMap.controlWristJoint()) > .1){
            return true;
        }
        else if(levelMode){
            return initialized && armSubsystem.isShoulderAtSetPoint(sPreset) && Math.abs(armSubsystem.getWristEncoderAngle() + armSubsystem.getShoulderEncoderAngle())<ConstantsMap.WRIST_TOLERANCE;
        }
        else{
            return initialized && armSubsystem.isShoulderAtSetPoint(sPreset) && armSubsystem.isWristAtSetPoint(wPreset);
        }
        
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("Preset Finished at: " + armSubsystem.getShoulderEncoderAngle() + " degress");

    }    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("Preset Interupt");
    }
}
