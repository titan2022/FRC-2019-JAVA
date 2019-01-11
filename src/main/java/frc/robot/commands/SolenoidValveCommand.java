/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SolenoidValveSubsystem;

import frc.robot.OI;
import frc.robot.XboxMap;
import frc.robot.ConstantsMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SolenoidValveCommand extends Command {
    SolenoidValveSubsystem solenoidValveSubsystem = Robot.solenoidValveSubsystem;
    XboxMap xboxMap = new XboxMap();
    OI oi = Robot.oi;
    private boolean in = false;
    private double lastPressed = 0;
    public SolenoidValveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(solenoidValveSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Solenoid init");
        //solenoidValveSubsystem.setLeftOff();
       // solenoidValveSubsystem.setRightOff();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(xboxMap.in() && (System.currentTimeMillis() - lastPressed) > 500) {
    		lastPressed = System.currentTimeMillis();
    		if(in) {
    			solenoidValveSubsystem.in();
    			in = !in;
    		}
    		else {
    			solenoidValveSubsystem.out();
    			in = !in;
    		}
    	}
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
