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
public class GrabberSubsystem extends Subsystem {
  private DoubleSolenoid topGrabPiston, hatchReleasePiston;
  public GrabberSubsystem() {
    topGrabPiston = new DoubleSolenoid(4, 5);
    hatchReleasePiston = new DoubleSolenoid(6, 7);
  }

  public void activateTopGrabPiston() {
    topGrabPiston.set(Value.kForward);
  }

  public void reverseTopGrabPiston() {
    topGrabPiston.set(Value.kReverse);
  }

  public void deactivateTopGrabPiston() {
    topGrabPiston.set(Value.kOff);
  }

  public void activateHatchReleasePiston() {
    hatchReleasePiston.set(Value.kForward);
  }

  public void reverseHatchReleasePiston() {
    hatchReleasePiston.set(Value.kReverse);
  }

  public void deactivateHatchReleasePiston() {
    hatchReleasePiston.set(Value.kOff);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
