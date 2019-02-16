package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ArmZero;
import frc.robot.commands.WristZero;

public class ControlPanelMap {
	static OI oi = Robot.oi;
	JoystickButton  
			rocketHatchPreset1,
			rocketHatchPreset2,
			rocketHatchPreset3,
			rocketBallPreset1,
			rocketBallPreset2,
			rocketBallPreset3,
			cargoHatchPreset,
			cargoBallPreset,
			ballCollectPreset,
			hatchCollectPreset,
			goHome,
			toggleDebug;

			
	public ControlPanelMap(){
		rocketBallPreset1 = new JoystickButton(oi.controlPanel1, 1);
		rocketBallPreset2 = new JoystickButton(oi.controlPanel1, 3);
		rocketBallPreset3 = new JoystickButton(oi.controlPanel1, 5);
		rocketHatchPreset1 = new JoystickButton(oi.controlPanel1, 2);
		rocketHatchPreset2 = new JoystickButton(oi.controlPanel1, 4);
		rocketHatchPreset3 = new JoystickButton(oi.controlPanel1, 6);

		cargoBallPreset = new JoystickButton(oi.controlPanel1, 7);
		cargoHatchPreset = new JoystickButton(oi.controlPanel1, 8);

		ballCollectPreset = new JoystickButton(oi.controlPanel1, 9);
		hatchCollectPreset = new JoystickButton(oi.controlPanel1,10);

		goHome = new JoystickButton(oi.controlPanel1, 11);
		toggleDebug = new JoystickButton(oi.controlPanel1, 12);
	}

	public void normalMode(){
		

		rocketHatchPreset1.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_1));
		rocketHatchPreset2.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_2));
		rocketHatchPreset3.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_3));
		rocketBallPreset1.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_1));
		rocketBallPreset2.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_2));
		rocketBallPreset3.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_3));

		cargoHatchPreset.whenPressed(new ArmPresetCommand(ConstantsMap.CARGO_HATCH_PRESET));
		cargoBallPreset.whenPressed(new ArmPresetCommand(ConstantsMap.CARGO_BALL_PRESET));

		ballCollectPreset.whenPressed(new ArmPresetCommand(ConstantsMap.BALL_COLLECT_PRESET));
		hatchCollectPreset.whenPressed(new ArmPresetCommand(ConstantsMap.HATCH_COLLECT_PRESET));

		goHome.whenPressed(new ArmPresetCommand(ConstantsMap.GO_HOME_PRESET));
	}
	public void debugMode(){
		rocketHatchPreset1.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_1));
		rocketHatchPreset2.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_2));
		rocketHatchPreset3.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_HATCH_PRESET_3));
		rocketBallPreset1.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_1));
		rocketBallPreset2.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_2));
		rocketBallPreset3.whenPressed(new ArmPresetCommand(ConstantsMap.ROCKET_BALL_PRESET_3));

		cargoHatchPreset.whenPressed(new ArmPresetCommand(ConstantsMap.CARGO_HATCH_PRESET));
		cargoBallPreset.whenPressed(new ArmPresetCommand(ConstantsMap.CARGO_BALL_PRESET));

		ballCollectPreset.whenPressed(new ArmZero());
		hatchCollectPreset.whenPressed(new WristZero());

		goHome.whenPressed(new ArmPresetCommand(ConstantsMap.GO_HOME_PRESET));
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