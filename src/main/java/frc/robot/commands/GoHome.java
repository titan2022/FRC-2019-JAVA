package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem2;

public class GoHome extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    boolean waitForWrist;
    public GoHome() {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {        
        if(armSubsystem.getShoulderEncoderAngle() < ConstantsMap.SHOULDER_WRIST_FOLD_ANGLE && armSubsystem.getWristEncoderAngle() < ConstantsMap.WRIST_MAX_ANGLE - 30){
            waitForWrist = true;
        }
        else{
            armSubsystem.setShoulderSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);
            waitForWrist = false;
        }
        armSubsystem.setWristSetPoint(ConstantsMap.WRIST_MAX_ANGLE);


    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {        
        if(waitForWrist){
            if(armSubsystem.getWristEncoderAngle() > ConstantsMap.WRIST_MAX_ANGLE-30){
                waitForWrist = false;
                armSubsystem.setShoulderSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);
            }
        }

    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return armSubsystem.isShoulderAtSetPoint() && armSubsystem.isWristAtSetPoint();
        
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("Go Home Completed Successfully");

    }    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("Go Home Intrrupted");
    }
}
