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
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * Add your docs here.
 */
public class GrabberSubsystem extends Subsystem 
{
  private DoubleSolenoid pushVelcro, pushHatch;
  private WPI_TalonSRX mecanumWheel, bottomRod;
  
  public GrabberSubsystem() {
    pushHatch = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_1, ConstantsMap.SOLENOID_PORT_2);
    pushVelcro = new DoubleSolenoid(ConstantsMap.SOLENOID_PORT_3, ConstantsMap.SOLENOID_PORT_4);
    mecanumWheel = new WPI_TalonSRX(RobotMap.MECANUM_GRABBER_PORT);  
    bottomRod = new WPI_TalonSRX(RobotMap.ROD_GRABBER_PORT);
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

  public void cargoCollect()
  {
    mecanumWheel.set(ConstantsMap.MECANUM_WHEEL_SPEED);  //clockwise
    bottomRod.set(-ConstantsMap.BOTTOM_ROD_SPEED);    //counterclockwise
  }

  public void cargoPush()
  {
    mecanumWheel.set(-ConstantsMap.MECANUM_WHEEL_SPEED);  //counterclockwise
    bottomRod.set(ConstantsMap.BOTTOM_ROD_SPEED);      //clockwise
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
