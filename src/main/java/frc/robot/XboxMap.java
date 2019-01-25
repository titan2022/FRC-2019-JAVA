/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class XboxMap {
	static OI oi = Robot.oi;
	
	//Drive commands
	static public boolean startAutoBrakerSystem(){
		return oi.xbox.getRightBumperValue();
	}
	
//	static public boolean controlDriverGear() {
//		return oi.xbox.getLeftBumperValue();
//	}
//	
	//Grabber commands 
//	static public double controlUpperGrabberOut(){
//		return oi.xbox.getRightTriggers();
//	}
//	
//	static public double controlUpperGrabberIn(){
//		return oi.xbox.getLeftTriggers();
//	}

	//Elevator commands
	static public double controlFrontElevator(){
		return oi.xbox.getRightY();
	}
	
	static public boolean startAutoElevatorBrakerSystem() {
		return oi.xbox.getStartValue();
	}

	//Universal stop command
	static public boolean stopSystem() {
		return oi.xbox.getBackValue();
	}
	
	static public boolean grabPistonVelcro() {
		return oi.xbox.getLeftBumperValue();
	}
	
	static public boolean grabPistonHatch() {
		return oi.xbox.getRightBumperValue();
	}

	static public boolean inTake() {
		return oi.xbox.getAValue();
	}

	static public boolean override() {
		return oi.xbox.getXValue();
	}
	
	static public boolean outTake() {
		return oi.xbox.getBValue();
	}

	static public double right() {
		return oi.xbox.getRightY();
	}
	
	static public double left() {
		return oi.xbox.getLeftY();
	}

	// static public boolean shiftLow() {
	// 	 return oi.ps4.getBValue();
	// }

	// static public boolean shiftHigh() {
	// 	return oi.ps4.getAValue();
	// }
	
	
	// static public double actuate() {
	// 	return oi.xbox.getLeftY();
	// }
	
	// static public double right() {
	// 	return oi.ps4.getRightY();
	// }
	
	// static public double left() {
	// 	return oi.ps4.getLeftY();
	// }
	//placeholder command
	

	static public boolean runFollowLineCommand() {
	 	return oi.xbox.getAValue();
	}
	

	//TODO B is also being used for outtake (Change later)
	static public boolean interruptFollowLine() {
		return oi.xbox.getBValue();
	}
}