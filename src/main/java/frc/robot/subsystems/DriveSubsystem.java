/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.ConstantsMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 */
public class DriveSubsystem extends Subsystem {

	private WPI_TalonSRX left1,left2,left3,right1,right2, right3;

	private AHRS ahrs;
	

	public DriveSubsystem() {
        System.out.println("Drive Subsystem Init");

		//Instantiate motors		
		left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_PORT_2);
		left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_PORT_1);
		right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_PORT_1);		
		right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_PORT_2);
		
		//Invert Motors
		
		right1.setInverted(true);
		right2.setInverted(true);
		//right3.setInverted(false);
		
		//Instantiate Encoders
		left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		
		//Instantiate Gyro | Gyro automatically calibrates when given power
        ahrs = new AHRS(SPI.Port.kMXP);
		stop();
		

		
		SmartDashboard.putData(ahrs);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new DriveCommand());
    }
	

	//sets the speed for both of the left motors
	public void setLeftSpeed(double speed) {
		left1.set(speed);
		left2.set(speed);
	}	
	
	//sets the speed for both of the right motors
	public void setRightSpeed (double speed) {
		right1.set(speed);
		right2.set(speed);	
	}
	
	public double getLeftSpeed() {		
		return left1.getSelectedSensorVelocity(0);
	}	
	
	public double getRightSpeed() {		
		return right1.getSelectedSensorVelocity(0);		
	}
	
	public void tankDrive(double leftSpeed, double rightSpeed){
		setLeftSpeed(leftSpeed);
		setRightSpeed(rightSpeed);
	}
	
	public void enableBrake(){
		//enable on left
		left1.setNeutralMode(NeutralMode.Brake);
		left2.setNeutralMode(NeutralMode.Brake);

		//enable on right
		right1.setNeutralMode(NeutralMode.Brake);
		right2.setNeutralMode(NeutralMode.Brake);
	}
	
	public void disableBrake(){
		//disable on left
		left1.setNeutralMode(NeutralMode.Coast);
		left2.setNeutralMode(NeutralMode.Coast);
		
		//disable on right
		right1.setNeutralMode(NeutralMode.Coast);
		right2.setNeutralMode(NeutralMode.Coast);
	}
	 

	//Get Encoder Distances
	public double getRightEncoderDistance(){
		return right1.getSelectedSensorPosition(0)* -1 * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
	}	
	public double getLeftEncoderDistance(){
		return left1.getSelectedSensorPosition(0)* -1 * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
	}
	
	//Get Encoder counts
	public int getLeftEncoderCount(){
		return left1.getSelectedSensorPosition(0)* -1;
	}	
	public int getRightEncoderCount(){
		return right1.getSelectedSensorPosition(0)* -1;
	}
	
	//Get Encoder Rates
	public double getRightEncoderRate(){
		return right1.getSelectedSensorVelocity(0)* -1;
	}	
	public double getLeftEncoderRate(){
		return left1.getSelectedSensorVelocity(0)* -1;
	}
	
	//reset encoders
	public void resetEncoders(){
		left1.getSensorCollection().setQuadraturePosition(0, 0);
		right1.getSensorCollection().setQuadraturePosition(0, 0);
	}
	
	public AHRS getGyro(){
		return ahrs;
	}
	
	public double getGyroAngle(){
		return ahrs.getAngle(); 
	}

	public void resetGyro() {
		ahrs.reset();
	}

	public void stop() {
		left1.set(0);
		left2.set(0);
		right1.set(0);
		right2.set(0);
	}
    
}