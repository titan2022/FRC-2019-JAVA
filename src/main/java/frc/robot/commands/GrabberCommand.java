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

public class GrabberCommand extends Command {
  OI oi = Robot.oi;
  XboxMap xboxMap = new XboxMap();
  private double grabPressed= 0;
  private double releasePressed = 0;
  private double grabCount=0;
  private double releaseCount = 0;
  public double milliTime = 500;
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
    if (XboxMap.grabPiston()==true)
    {
      grabberSubsystem.activateTopGrabPiston();
      grabCount++;
    } 
   if((grabCount==1) && (System.currentTimeMillis() - grabPressed) > milliTime) 
      {
        grabPressed = System.currentTimeMillis();
        grabCount++;
       grabberSubsystem.reverseTopGrabPiston();
      }
    if((grabCount==2) &&(System.currentTimeMillis() - grabPressed > milliTime))
    {
      grabPressed = System.currentTimeMillis();
      grabberSubsystem.deactivateTopGrabPiston();
      grabCount=0;
    }


    if (XboxMap.releasePiston()==true)
     {
      grabberSubsystem.activateHatchReleasePiston();
      releaseCount++;
     } 
   if((releaseCount==1) && (System.currentTimeMillis() - releasePressed) > milliTime) 
      {
        releasePressed = System.currentTimeMillis();
        releaseCount++;
       grabberSubsystem.reverseHatchReleasePiston();
      }
    if((releaseCount==2) &&(System.currentTimeMillis() - releasePressed > milliTime))
      {
      releasePressed = System.currentTimeMillis();
      grabberSubsystem.deactivateHatchReleasePiston();
      releaseCount=0;
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
