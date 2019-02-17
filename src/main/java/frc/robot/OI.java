/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ArmZero;
import frc.robot.commands.WristZero;

/**
* This class is the glue that binds the controls on the physical operator
* interface to the commands and command groups that allow control of the robot.
*/
public class OI {
	//User interface Constants
	public double attackThrottleSensitivity=.1;
	//Controllers
	public static XboxController xbox,ps4;
	public static Joystick controlPanel1,controlPanel2;
	public JoystickButton  
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
	
	public OI(){
		xbox = new XboxController(0);
		controlPanel1 = new Joystick(1);
		controlPanel2 = new Joystick(2);
		
		//attack3_L = new Attack3(3);
		//attack3_R = new Attack3(4);
		rocketBallPreset1 = new JoystickButton(controlPanel1, 5);
		rocketBallPreset2 = new JoystickButton(controlPanel1, 7);
		rocketBallPreset3 = new JoystickButton(controlPanel1, 9);
		rocketHatchPreset1 = new JoystickButton(controlPanel1, 6);
		rocketHatchPreset2 = new JoystickButton(controlPanel1, 8);
		rocketHatchPreset3 = new JoystickButton(controlPanel1, 10);

		cargoBallPreset = new JoystickButton(controlPanel1, 3);
		cargoHatchPreset = new JoystickButton(controlPanel1, 4);

		ballCollectPreset = new JoystickButton(controlPanel1, 1);
		hatchCollectPreset = new JoystickButton(controlPanel1,2);

		goHome = new JoystickButton(controlPanel1, 12);
		toggleDebug = new JoystickButton(controlPanel1, 11);
		
		normalMode();

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
		

		rocketHatchPreset1.whenPressed(null);
		rocketHatchPreset2.whenPressed(null);
		rocketHatchPreset3.whenPressed(null);
		rocketBallPreset1.whenPressed(null);
		rocketBallPreset2.whenPressed(null);
		rocketBallPreset3.whenPressed(null);

		cargoHatchPreset.whenPressed(null);
		cargoBallPreset.whenPressed(null);

		ballCollectPreset.whenPressed(new WristZero());
		hatchCollectPreset.whenPressed(new ArmZero());

		goHome.whenPressed(new ArmPresetCommand(ConstantsMap.GO_HOME_PRESET));
	}
}