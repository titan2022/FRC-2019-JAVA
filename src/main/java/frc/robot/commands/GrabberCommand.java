/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Servo;

public class GrabberCommand extends Command {
  OI oi = Robot.oi;
  XboxMap xboxMap = new XboxMap();
  private double lastPressedVelcro= 0;
  private double lastPressedHatch= 0;
  public double milliTime = 500;
  public boolean pistonClosedVelcro = true;
  public boolean pistonClosedHatch = true;


  GrabberSubsystem grabberSubsystem = Robot.grabberSubsystem;
  public GrabberCommand() 
  {
    requires (grabberSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // solenoids activated and deactivated based on input
    if (XboxMap.grabPistonVelcro() && System.currentTimeMillis() - lastPressedVelcro > milliTime)
    {
        lastPressedVelcro = System.currentTimeMillis();
        if (pistonClosedVelcro)
        {
            grabberSubsystem.velcroPushOut();
            pistonClosedVelcro = false;
        }
        else
        {
            grabberSubsystem.velcroPushIn();
            pistonClosedVelcro = true;
        }
    }
    
    if(XboxMap.grabPistonHatch() && System.currentTimeMillis() - lastPressedHatch > milliTime)
    {
        lastPressedHatch = System.currentTimeMillis();
        if (pistonClosedHatch)
        {
          grabberSubsystem.hatchPushOut();
          pistonClosedHatch = false;
        }
        else
        {
          grabberSubsystem.hatchPushIn();
          pistonClosedHatch = true;
        }
    }

    // ball collector runs based on input
    if (XboxMap.grabberInControl() && !XboxMap.grabberOutControl() && !grabberSubsystem.isRotating)
    {
       grabberSubsystem.ballGrabIn();
    }

    if (XboxMap.grabberOutControl() && !XboxMap.grabberInControl() && !grabberSubsystem.isRotating)
    {
       grabberSubsystem.ballPushOut();
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