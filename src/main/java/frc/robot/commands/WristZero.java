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

public class WristZero extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    private double shoulderAngle;
    private double wristAngle;
    private double actualShoulderAngle;
    private double actualWristAngle;
    public WristZero() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Wrist Zero Started");
        armSubsystem.setWristJointSpeed(.62);      
        
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
       
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

        armSubsystem.setShoulderJointSpeed(0);
        armSubsystem.setWristJointSpeed(0);;

    }
}