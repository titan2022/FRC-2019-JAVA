/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
* Add your docs here.
*/
public class HatchGrabberSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    //DoubleSolenoid grabberPiston, releasePiston1, releasePiston2;
    Solenoid grabberPiston, releasePiston1, releasePiston2;
    public HatchGrabberSubsystem() {
        
        /* grabberPiston = new DoubleSolenoid(RobotMap.GRAB_PISTON_F, RobotMap.GRAB_PISTON_R);
        releasePiston1 = new DoubleSolenoid(RobotMap.RELEASE_PISTON_1_F, RobotMap.RELEASE_PISTON_1_R); */
        grabberPiston = new Solenoid(4);
        //releasePiston1 = new DoubleSolenoid(4, 5);
        releasePiston1 = new Solenoid(5);
    }

    public void extendGrabber() {
        grabberPiston.set(false);
    }

    public void retractGrabber() {
        grabberPiston.set(true);
    }

    public void releaseHatch() {
        releasePiston1.set(false);
    }

    public void retractEjectors() {
        releasePiston1.set(true);
    }
    
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
