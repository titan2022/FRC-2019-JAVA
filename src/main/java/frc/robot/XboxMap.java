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
	public static boolean startAutoBrakerSystem(){
		return oi.xbox.getRightBumperValue();
	}
	
//	public static boolean controlDriverGear() {
//		return oi.xbox.getLeftBumperValue();
//	}
//	
	//Grabber commands 
//	public static double controlUpperGrabberOut(){
//		return oi.xbox.getRightTriggers();
//	}
//	
//	public static double controlUpperGrabberIn(){
//		return oi.xbox.getLeftTriggers();
//	}

	//Elevator commands
	public static double controlFrontElevator(){
		return oi.xbox.getRightY();
	}
	
	public static boolean startAutoElevatorBrakerSystem() {
		return oi.xbox.getStartValue();
	}

	//Universal stop command
	public static boolean stopSystem() {
		return oi.xbox.getBackValue();
	}
	
	public static boolean piston() {
		return oi.xbox.getLeftBumperValue();
	}
	
	public static boolean inTake() {
		return oi.xbox.getAValue();
	}

	public static boolean override() {
		return oi.xbox.getXValue();
	}
	
	public static boolean outTake() {
		return oi.xbox.getBValue();
	}

	public static double right() {
		return oi.xbox.getRightY();
	}
	
	public static double left() {
		return oi.xbox.getLeftY();
	}

	public static boolean runFollowLineCommand() {
		return oi.xbox.getXValue();
	}

	// public static boolean shiftLow() {
	// 	 return oi.ps4.getBValue();
	// }

	// public static boolean shiftHigh() {
	// 	return oi.ps4.getAValue();
	// }
	
	
	// public static double actuate() {
	// 	return oi.xbox.getLeftY();
	// }
	
	// public static double right() {
	// 	return oi.ps4.getRightY();
	// }
	
	// public static double left() {
	// 	return oi.ps4.getLeftY();
	// }

}