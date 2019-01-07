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
		return oi.xbox.getRightBumperValue();
	}
	
//	public boolean controlDriverGear() {
//		return oi.xbox.getLeftBumperValue();
//	}
//	
	//Grabber commands 
//	public double controlUpperGrabberOut(){
//		return oi.xbox.getRightTriggers();
//	}
//	
//	public double controlUpperGrabberIn(){
//		return oi.xbox.getLeftTriggers();
//	}

	//Elevator commands
	public double controlFrontElevator(){
		return oi.xbox.getRightY();
	}
	
	public boolean startAutoElevatorBrakerSystem() {
		return oi.xbox.getStartValue();
	}

	//Universal stop command
	public boolean stopSystem() {
		return oi.xbox.getBackValue();
	}
	
	public boolean piston() {
		return oi.xbox.getLeftBumperValue();
	}
	
	public boolean inTake() {
		return oi.xbox.getAValue();
	}
	public boolean override() {
		return oi.xbox.getXValue();
	}
	
	public boolean outTake() {
		return oi.xbox.getBValue();
	}
	public boolean shiftLow() {
		return oi.ps4.getBValue();
	}
	public boolean shiftHigh() {
		return oi.ps4.getAValue();
	}
	
	
	public double actuate() {
		return oi.xbox.getLeftY();
	}
	
	public double right() {
		return oi.ps4.getRightY();
	}
	
	public double left() {
		return oi.ps4.getLeftY();
	}
}