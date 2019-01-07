/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Xbox extends Joystick{

	//Axes
	private static final int LEFT_XAXIS = 0;
	private static final int LEFT_YAXIS = 1;
	private static final int RIGHT_XAXIS = 4;
	private static final int RIGHT_YAXIS = 5;
	private static final int TRIGGERS_LEFT = 2;
	private static final int TRIGGERS_RIGHT = 3;
	//Buttons
	private static final int ABUTTON = 1;
	private static final int BBUTTON = 2;
	private static final int XBUTTON = 3;
	private static final int YBUTTON = 4;
	private static final int LEFT_BUMPER = 5;
	private static final int RIGHT_BUMPER = 6;
	private static final int BACK_BUTTON = 7;
	private static final int START_BUTTON = 8;

	/**
	 * Constructor creates an object to interface with an Xbox 360 controller
	 *
	 * @param port The port on the drivers station that the controller is
	 * connected to
	 * @return
	 */
	public Xbox(int port) {
		super(port);
	}

	/**
	 * Get the value of X of the left joystick. The right side of the axis is
	 * positive.
	 *
	 * @param
	 * @return The value of the axis from -1 to 1.
	 */
	public double getLeftX() {
		return (getRawAxis(LEFT_XAXIS));
	}

	/**
	 * Get the value of Y of the left joystick. The upward part of the axis is
	 * positive.
	 *
	 * @param
	 * @return The value of the axis from -1 to 1.
	 */
	public double getLeftY() {
		return (1 * getRawAxis(LEFT_YAXIS));
		//Multiply output by -1 because GetAxis() returns the axis backwards.
	}

	/**
	 * Get the value of X of the right joystick. The right side of the axis is
	 * positive.
	 *
	 * @param
	 * @return The value of the axis from -1 to 1.
	 */
	public double getRightX() {
		return (getRawAxis(RIGHT_XAXIS));
	}

	/**
	 * Get the value of Y of the right joystick. The upward part of the axis is
	 * positive.
	 *
	 * @return The value of the axis from -1 to 1.
	 */
	public double getRightY() {
		return (-1 * getRawAxis(RIGHT_YAXIS));
	}

	/**
	 * Get the value corresponding to the triggers. The left stick adds to it,
	 * the right stick subtracts. Range -1 to 1.
	 *
	 * @param
	 * @return The value corresponding to the triggers.
	 */
	public double getRightTriggers() {
		return (getRawAxis(TRIGGERS_RIGHT));
	}
	
	public double getLeftTriggers() {
		return (getRawAxis(TRIGGERS_LEFT));
	}

	/**
	 * Get the current state of the A button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getAValue() {
		return (getRawButton(1));
	}

	/**
	 * Get the current state of the B button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getBValue() {
		return (getRawButton(2));
	}

	/**
	 * Get the current state of the X button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getXValue() {
		return (getRawButton(3));
	}

	/**
	 * Get the current state of the Y button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getYValue() {
		return (getRawButton(4));
	}

	/**
	 * Get the current state of the Left Bumper.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getLeftBumperValue() {
		return (getRawButton(5));
	}

	/**
	 * Get the current state of the Right Bumper.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getRightBumperValue() {
		return (getRawButton(6));
	}

	/**
	 * Get the current state of the Back button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getBackValue() {
		return (getRawButton(7));
	}

	/**
	 * Get the current state of the Start button.
	 *
	 * @param
	 * @return The current state of the button
	 */
	public boolean getStartValue() {
		return (getRawButton(8));
	}

	/**
	 * Get the angle of the left joystick. It is counter-clockwise, 0 is down.
	 *
	 * @param reflectCircle Choose whether to reflect over the Y axis.
	 * @return The angle of the left joystick.
	 */
	public double getLeftAngle(boolean reflectCircle) {
		float yAxis = (float) (-1 * getLeftY());
		float xAxis;

		if (reflectCircle) {
			xAxis = (float) getLeftX();
		} else {
			xAxis = (float) (-1 * getLeftX());
		}

		double angle = 0;

		if (xAxis == 0 && yAxis == 0) {
			angle = 0;
		} else if (xAxis == 0) {
			if (yAxis > 0) {
				angle = 0;
			} else {
				angle = Math.PI; //Legacy: Math.PI
			}
		} else if (yAxis == 0) {
			if (xAxis > 0) {
				angle = (3 * Math.PI / 2);
			} else {
				angle = ((Math.PI) / 2);
			}
		} else {
			angle = Math.atan2(yAxis, xAxis);
		}

		if (angle >= (2 * Math.PI)) {
			angle = angle - (2 * Math.PI);
		}

		if (angle < 0) {
			angle = angle + (2 * Math.PI);
		}

		return (angle);
	}

	/**
	 * Get the angle of the right joystick. It is counter-clockwise, 0 is down.
	 *
	 * @param reflectCircle Choose whether to reflect over the Y axis.
	 * @return The angle of the right joystick.
	 */
	public double getRightAngle(boolean reflectCircle) {
		float yAxis = (float) (-1 * getRightY());
		float xAxis;

		if (reflectCircle) {
			xAxis = (float) getRightX();
		} else {
			xAxis = (float) (-1 * getRightX());
		}

		double angle = 0;

		if (xAxis == 0 && yAxis == 0) {
			angle = 0;
		} else if (xAxis == 0) {
			if (yAxis > 0) {
				angle = 0;
			} else {
				angle = Math.PI;
			}
		} else if (yAxis == 0) {
			if (xAxis > 0) {
				angle = (3 * Math.PI / 2);
			} else {
				angle = ((Math.PI) / 2);
			}
		} else {
			angle = Math.atan2(yAxis, xAxis);
		}

		if (angle >= (2 * Math.PI)) {
			angle = angle - (2 * Math.PI);
		}

		if (angle < 0) {
			angle = angle + (2 * Math.PI);
		}

		return (angle);
	}

	/**
	 * Get the magnitude of the left joystick. Value is between 0 and 1.
	 *
	 * @param
	 * @return The magnitude of the left joystick.
	 */
	public double getLeftMagnitude() {
		return (limit(Math.sqrt(Math.pow(getLeftX(), (2)) + Math.pow(getLeftY(), (2)))));
	}

	/**
	 * Get the magnitude of the right joystick. Value is between 0 and 1.
	 *
	 * @param
	 * @return The magnitude of the right joystick.
	 */
	public double getRightMagnitude() {
		return (limit(Math.sqrt(Math.pow(getRightX(), (2)) + Math.pow(getRightY(), (2)))));
	}

	////////////////Get Raw Button Methods to be used by Command/Subsystem Interface
	/**
	 * Get A Button
	 *
	 * @param
	 * @return
	 * @return A Button
	 */
	public JoystickButton getAButton() {
		return (new JoystickButton(this, ABUTTON));
	}

	/**
	 * Get B Button
	 *
	 * @param
	 * @return
	 * @return B Button
	 */
	public JoystickButton getBButton() {
		return (new JoystickButton(this, BBUTTON));
	}

	/**
	 * Get X Button
	 *
	 * @param
	 * @return X Button
	 */
	public JoystickButton getXButton() {
		return (new JoystickButton(this, XBUTTON));
	}

	/**
	 * Get Y Button
	 *
	 * @param
	 * @return Y Button
	 */
	public JoystickButton getYButton() {
		return (new JoystickButton(this, YBUTTON));
	}

	/**
	 * Get Back Button
	 *
	 * @param
	 * @return Back Button
	 */
	public JoystickButton getBackButton() {
		return (new JoystickButton(this, BACK_BUTTON));
	}

	/**
	 * Get Start Button
	 *
	 * @param
	 * @return Start Button
	 */
	public JoystickButton getStartButton() {
		return (new JoystickButton(this, START_BUTTON));
	}

	/**
	 * Get Left Bumper
	 *
	 * @param
	 * @return Left Bumper
	 */
	public JoystickButton getLeftBumper() {
		return (new JoystickButton(this, LEFT_BUMPER));
	}

	/**
	 * Get Right Bumper
	 *
	 * @param
	 * @return Right Bumper
	 */
	public JoystickButton getRightBumper() {
		return (new JoystickButton(this, RIGHT_BUMPER));
	}

	////////////////End Raw Button Methods for Command/Subsystem Interface
	/**
	 * Get the buttons of the controller, to be used by other functions.
	 *
	 * The buttons on the controller follow this mapping 1: A 2: B 3: X 4: Y 5:
	 * Left Bumper 6: Right Bumper 7: Back 8: Start 9: Left Joystick 10: Right
	 * Joystick
	 *
	 * @param button The button to get the value of. Range 1-10.
	 * @return The state of the button.
	 */
	public boolean getRawButton(int button) {
		return (super.getRawButton(button));
	}

	/**
	 * Get the raw axes of the controller, to be used by other functions.
	 *
	 * The axis on the controller follow this mapping 1: Left Stick X Axis
	 * -Left:Negative ; Right: Positive 2: Left Stick Y Axis -Up: Negative ;
	 * Down: Positive 3: Triggers -Left: Positive ; Right: Negative 4: Right
	 * Stick X Axis -Left: Negative ; Right: Positive 5: Right Stick Y Axis -Up:
	 * Negative ; Down: Positive 6: Directional Pad (Not recommended, buggy)
	 *
	 * @param axis The axis to get the value of. Range 1-6.
	 * @return The value of the axis from -1 to 1.
	 */
	public double getRawAxis(int axis) {
		return super.getRawAxis(axis);
	}

	/**
	 * Normalize outputs to be between -1 and 1.
	 *
	 * @param num The value to normalize.
	 * @return The normalized value.
	 */
	public double limit(double num) {
		if (num > 1) {
			num = 1;
		} else if (num < -1) {
			num = -1;
		}
		return num;
	}
}