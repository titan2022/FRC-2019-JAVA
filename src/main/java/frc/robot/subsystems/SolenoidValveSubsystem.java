/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class SolenoidValveSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private Solenoid in,out;
    private DoubleSolenoid left, right;
    public SolenoidValveSubsystem(){
        //left = new DoubleSolenoid(RobotMap.LEFT_SOLENOID_PORT_A,RobotMap.LEFT_SOLENOID_PORT_B);
        in = new Solenoid(6);
        out = new Solenoid(7);       
        //right = newDoubleSolenoid(RobotMap.RIGHT_SOLENOID_PORT_A,RobotMap.RIGHT_SOLENOID_PORT_B);
    }
    public void in() {
    	in.set(true);
        out.set(false);
    }
    public void out() {
    	in.set(false);
    	out.set(true);
    	
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public void setLeftForward(){
        left.set(Value.kForward);
    }
    public void setRightForward(){
        right.set(Value.kForward);
    }
    public void setLeftReverse(){
        left.set(Value.kReverse);
    }
    public void setRightReverse(){
        right.set(Value.kReverse);
    }
    public void setLeftOff(){
        left.set(Value.kOff);
    }
    public void setRightOff(){
        right.set(Value.kOff);
    }
}
