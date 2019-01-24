                                                                                                                                                                                                                                                                                                                                  /*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class GrabberSubsystem extends Subsystem 
{
  private DoubleSolenoid topGrabPiston, hatchReleasePiston;
 
  public GrabberSubsystem() {
    topGrabPiston = new DoubleSolenoid(4, 5);
    hatchReleasePiston = new DoubleSolenoid(6, 7);
  }

  public void activateGrab() {
    topGrabPiston.set(Value.kForward);
  }

  public void reverseGrab() {
    topGrabPiston.set(Value.kReverse);
  }

  public void deactivateGrab() {
    topGrabPiston.set(Value.kOff);
  }

  public void activateRelease() {
    hatchReleasePiston.set(Value.kForward);
  }

  public void reverseRelease() {
    hatchReleasePiston.set(Value.kReverse);
  }

  public void deactivateRelease() {
    hatchReleasePiston.set(Value.kOff);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
