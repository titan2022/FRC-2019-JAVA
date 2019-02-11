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

	static public boolean runFollowLineCommand() {
	 	return oi.xbox.getAValue();
	}
	public static boolean startAutoBrakerSystem(){
		return oi.xbox.getRightBumperValue();
	}
	//TODO B is also being used for outtake (Change later) // Put it on the other controller??
	
	public static double controlWristJoint() {
		return oi.xbox.getRightY();
	}
	
	public static double controlShoulderJoint() {
		return oi.xbox.getLeftY();
	}
	public static boolean toggleArmControl() {
		return oi.xbox.getXValue();
	}

	public static boolean enableWristLevelling() {
		return oi.xbox.getBValue();
	}

	//TODO Map to Buttons
	public static boolean enableCargoPreset() {
		return true;
	}

	public static boolean enableLevel2Preset() {
		return true;
	}

	public static boolean enableLevel3Preset() {
		return true;
	}

	
	public static double right() {
		return oi.xbox.getRightY();
	}
	
	public static double left() {
		return oi.xbox.getLeftY();
	}

	
}