package frc.robot;

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
		return oi.controlPanel1.getRawButtonPressed(11);
	}
	public static boolean toggleLevel() {
		return oi.controlPanel1.getRawButtonPressed(4);
	}
	public static boolean switchCam() {
		return oi.controlPanel1.getRawButtonPressed(4);
	}
	public static boolean toggleManual() {	
		return oi.controlPanel1.getRawButtonPressed(9);
	}
	public static boolean forceZero() {	
		return oi.controlPanel1.getRawButtonPressed(3);
	}
	public static boolean stopClimb(){
		return OI.controlPanel1.getRawButtonPressed(5);
	}
}