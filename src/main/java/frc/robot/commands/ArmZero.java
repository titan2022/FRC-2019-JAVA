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

public class ArmZero extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;
    private double shoulderAngle;
    private double wristAngle;
    private double actualShoulderAngle;
    private double actualWristAngle;
    public ArmZero() {
        requires(armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Arm Zero Started");
        armSubsystem.setShoulderJointSpeed(-.1);      
        
         
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
       
    }
    
    

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return armSubsystem.getShoulderLowerLimit();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        armSubsystem.setShoulderJointSpeed(0); 
        System.out.println("Arm Zero Command Stopped");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("Arm Zero Command Intrupted");

        armSubsystem.setShoulderJointSpeed(0);
        armSubsystem.setWristJointSpeed(0);;

    }
}