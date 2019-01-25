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
import frc.robot.ConstantsMap;


/**
 * Add your docs here.
 */
public class GrabberSubsystem extends Subsystem 
{
  private DoubleSolenoid pushVelcro, pushHatch;
 
  public GrabberSubsystem() {
    pushHatch = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_1, ConstantsMap.SOLENOID_PORT_2);
    pushVelcro = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_3, ConstantsMap.SOLENOID_PORT_4);
  }

  public void hatchPushOut() {
    pushHatch.set(Value.kForward);
  }

  public void hatchPushIn() {
    pushHatch.set(Value.kReverse);
  }

  public void velcroPushOut() {
    pushVelcro.set(Value.kForward);
  }

  public void velcroPushIn() {
    pushVelcro.set(Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
