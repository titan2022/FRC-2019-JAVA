/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ConstantsMap;
import frc.robot.RobotMap;
import frc.robot.commands.HatchGrabberCommand;

/**
* Add your docs here.
*/
public class HatchGrabberSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    //DoubleSolenoid grabberPiston, releasePiston1, releasePiston2;
    DoubleSolenoid grabberPiston, releasePiston1, releasePiston2;
    TalonSRX motor1,motor2;
    public HatchGrabberSubsystem() {
        
        grabberPiston = new DoubleSolenoid(RobotMap.HATCH_PISTON_F, RobotMap.HATCH_PISTON_R);
        releasePiston1 = new DoubleSolenoid(RobotMap.RELEASE_PISTON_1_F, RobotMap.RELEASE_PISTON_1_R); 
        //grabberPiston = new Solenoid(0,4);
        //releasePiston1 = new Solenoid(0,6);

        //releasePiston1 = new DoubleSolenoid(4, 5);
        //releasePiston1 = new Solenoid(5);

        motor1 = new TalonSRX(RobotMap.GRAB_SPIN_1);
        motor2 = new TalonSRX(RobotMap.GRAB_SPIN_2);
        motor1.setInverted(true);

    }

    public void extendHatch() {
        grabberPiston.set(Value.kForward);
    }
    
    public void retractHatch() {
        grabberPiston.set(Value.kReverse);
    }

    public void ejectHatch() {
        releasePiston1.set(Value.kReverse);
    }
    public void unEjectHatch() {
        releasePiston1.set(Value.kForward);

    }
    public void intakeWheels(){
        motor1.set(ControlMode.PercentOutput, ConstantsMap.BALL_INTAKE_SPEED);
        motor2.set(ControlMode.PercentOutput, ConstantsMap.BALL_INTAKE_SPEED);

    }
    public void outakeWheels(){
        motor1.set(ControlMode.PercentOutput, -ConstantsMap.BALL_OUTTAKE_SPEED);
        motor2.set(ControlMode.PercentOutput, -ConstantsMap.BALL_OUTTAKE_SPEED);

    }
    public void stopWheels(){
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
    
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new HatchGrabberCommand());
    }
}
