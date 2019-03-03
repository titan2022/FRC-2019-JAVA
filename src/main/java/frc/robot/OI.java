package frc.robot;

import java.lang.reflect.Field;
import java.util.Vector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger.ButtonScheduler;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.GoToDistance;
import frc.robot.commands.GoToLine;
import frc.robot.pids.TurnToAngle;


public class OI {
	//User interface Constants
	public double attackThrottleSensitivity=.1;
	//Controllers
	public static Xbox xbox,ps4;
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
		xbox = new Xbox(0);
		normalMode();

	}
	public void normalMode(){
		JoystickButton xboxB = new JoystickButton(xbox, 2);
		xboxB.whenPressed(new GoToLine());
	}
	public void debugMode(){
	}

	public void unbindButtons() {
		Field schedulerBtns;
		try {
			schedulerBtns = Scheduler.class.getDeclaredField("m_buttons");
			schedulerBtns.setAccessible(true);
			((Vector<ButtonScheduler>) schedulerBtns.get(Scheduler.getInstance())).clear();
		} catch (Exception e) {
			System.err.println("wpilib broke");
			return;
		}
	}
}