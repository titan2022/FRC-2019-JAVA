/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class XboxMap {
	static OI oi = Robot.oi;
	
	//Driving Controls 
	public static double right() {
		return oi.xbox.getY(Hand.kRight);
	}
	
	public static double left() {
		return oi.xbox.getY(Hand.kLeft);
	}
	public static boolean toggleBrakes(){
		return oi.xbox.getBumperPressed(Hand.kRight);
	}

	
}