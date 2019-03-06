/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;



public class XboxMap {
	static OI oi = Robot.oi;
	
	//Driving Controls 
	public static double right() {
		return -oi.xbox.getY(Hand.kRight);
	}
	
	public static boolean switchCam() {
		return oi.xbox.getAButton();
	}
	public static double left() {
		return oi.xbox.getY(Hand.kLeft);
	}
	public static boolean toggleBrakes(){
		return oi.xbox.getBumperPressed(Hand.kRight);
	}
	public static void startRumble(){
		oi.xbox.setRumble(RumbleType.kLeftRumble, 1);
		oi.xbox.setRumble(RumbleType.kRightRumble, 1);
	}
	public static void stopRumble(){
		oi.xbox.setRumble(RumbleType.kLeftRumble, 0);
		oi.xbox.setRumble(RumbleType.kRightRumble, 0);
	}
	
}