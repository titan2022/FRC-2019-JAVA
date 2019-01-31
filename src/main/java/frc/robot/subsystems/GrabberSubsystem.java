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
import edu.wpi.first.wpilibj.PWMTalonSRX;;


/**
 * Add your docs here.
 */
public class GrabberSubsystem extends Subsystem 
{
  private DoubleSolenoid pushVelcro, pushHatch;
  private PWMTalonSRX mecanumWheel, bottomRod;
  private double wheelSpeed= 0.5; /*Hello fellow encoders.  
  Please regard this value as a placeholder.  This value may be changed if necessary 
  */
  public GrabberSubsystem() {
    pushHatch = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_1, ConstantsMap.SOLENOID_PORT_2);
    pushVelcro = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_3, ConstantsMap.SOLENOID_PORT_4);
    mecanumWheel = new PWMTalonSRX(ConstantsMap.SERVO_PORT_1);  
    bottomRod = new PWMTalonSRX(ConstantsMap.SERVO_PORT_2);
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

  public void ballGrabIn()
  {
    mecanumWheel.set(wheelSpeed);
  }

  public boolean isRotating()
  {
    if(mecanumWheel.get()==wheelSpeed)
    {
      return true;
    }
    return false;
  }

  public void ballPushOut()
  {
    mecanumWheel.set(-wheelSpeed);
    
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
