package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;

import frc.robot.subsystems.ArmSubsystem2;

public class WristForceZero extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    public WristForceZero() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Wrist Zero Started");
        armSubsystem.setWristJointSpeed(.6);      
        //armSubsystem.setWristSetPoint(ConstantsMap.WRIST_MAX_ANGLE);
        armSubsystem.setShoulderSetPoint(armSubsystem.getShoulderEncoderAngle());
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
       //if(armSubsystem.isWristAtSetPoint()){
       //    armSubsystem.setWristJointSpeed(.4);
       //}
    }
    
    

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return armSubsystem.getWristLowerLimit();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        armSubsystem.setWristJointSpeed(0); 
        System.out.println("Zero wrist Command Stopped");
        

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("Zero Wrist Command Intrupted");

        armSubsystem.setWristJointSpeed(0);;

    }
}