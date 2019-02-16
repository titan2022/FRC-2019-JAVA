package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ArmZero;
import frc.robot.commands.WristZero;

public class ControlPanelMap {
	static OI oi = Robot.oi;
	

			
	public ControlPanelMap(){
		
	}

	
	
	public static double controlWristJoint() {
		return oi.controlPanel2.getX();
	}
	
	public static double controlShoulderJoint() {
		return -oi.controlPanel1.getX();
	}
	public static boolean inTake() {
		return oi.controlPanel2.getRawButton(2);
	}
	public static boolean outTake() {
		return oi.controlPanel2.getRawButton(1);
	}
	public static boolean setBallMode() {
		return oi.controlPanel2.getRawButtonPressed(3);
	}
	public static boolean setHatcheMode() {
		return oi.controlPanel2.getRawButtonPressed(4);
	}
	public static boolean toggleDebug() {
		return oi.controlPanel1.getRawButtonPressed(2);
	}

/* 
	public static boolean toggleArmControl() {
		return oi.xbox.getXButtonPressed();
	}
	public static boolean toggleArmManualControl() {
		return oi.xbox.getAButtonPressed();
	}
	public static boolean zeroWrist() {
		return oi.xbox.getBackButtonPressed();
	}
	public static boolean zeroShoulder() {
		return oi.xbox.getStartButtonPressed();
	}
	public static boolean enableWristLevelling() {
		return oi.xbox.getBButtonPressed();
	}


	public static boolean hatchPiston() {
		return oi.xbox.getLeftBumperValue();
	}
	public static boolean grabberPiston() {
		return oi.xbox.getRightBumperValue();
	} */

}