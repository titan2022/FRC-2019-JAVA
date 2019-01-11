/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class XboxMap {
	OI oi = Robot.oi;
	
	//Drive commands
	public boolean startAutoBrakerSystem(){
		return oi.xbox.getStartValue();
	}
	
//	public boolean controlDriverGear() {
//		return oi.xbox.getLeftBumperValue();
//	}
//	
//	Solenoid  for prototyping
	public boolean in(){
		return oi.xbox.getXValue();
	}
	
//Grabber commands (no)
//	public double controlUpperGrabberOut(){
//		return oi.xbox.getRightTriggers();
//	}
//	
//	public double controlUpperGrabberIn(){
//		return oi.xbox.getLeftTriggers();
//	}

	//Elevator commands
	
	//shifted to Solenoid
	// public boolean piston() {
	// 	return oi.xbox.getLeftBumperValue();
	// }
	
	// public boolean inTake() {
	// 	return oi.xbox.getAValue();
	// }
	// public boolean override() {
	// 	return oi.xbox.getXValue();
	// }
	
	// public boolean outTake() {
	// 	return oi.xbox.getBValue();
	// }
	
	public double right() {
		return oi.xbox.getRightY();
	}
	
	public double left() {
		return oi.xbox.getLeftY();
	}

	
}