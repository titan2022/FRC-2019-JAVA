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
	private Encoder leftEncoder, rightEncoder;

	private AHRS ahrs;
	
	private Solenoid shift_high,shift_low;

	public DriveSubsystem() {
		//Instantiate motors		
		left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_PORT_1);
		left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_PORT_2);
		right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_PORT_1);		
		right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_PORT_2);
		
		//Invert Motors
		left1.setInverted(false);
		left2.setInverted(false);
		//left3.setInverted(false);
		right1.setInverted(false);
		right2.setInverted(false);
		//right3.setInverted(false);

		//Instantiate Encoders
		leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_PORT_A, RobotMap.LEFT_ENCODER_PORT_B, false);
		rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_PORT_A, RobotMap.RIGHT_ENCODER_PORT_B,true);
		
		//Instantiate Gyro | Gyro automatically calibrates when given power
        ahrs = new AHRS(SPI.Port.kMXP);
		stop();
		
		
//		//Set encoder distance per pulse
		leftEncoder.setDistancePerPulse(ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK);
		rightEncoder.setDistancePerPulse(ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK);
		SmartDashboard.putData(ahrs);
		SmartDashboard.putData(leftEncoder);
		SmartDashboard.putData(rightEncoder);
		
		shift_high = new Solenoid(2);
		shift_low = new Solenoid(3);

	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new DriveCommand());
    }
    
    public void shiftHigh() {
    	shift_high.set(true);
    	shift_low.set(false);
    }
    public void shiftLow() {
    	shift_high.set(false);
    	shift_low.set(true);
    }
   
   
	public void setLeftSpeed(double speed) {
			left1.set(speed);
			left2.set(speed);
			//left3.set(speed);
	}	
	
	public void setRightSpeed (double speed) {
			right1.set(speed);
			right2.set(speed);		
		//	right3.set(speed);
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
		left1.setNeutralMode(NeutralMode.Brake);
		left2.setNeutralMode(NeutralMode.Brake);
	//	left3.setNeutralMode(NeutralMode.Brake);
		right1.setNeutralMode(NeutralMode.Brake);
		right2.setNeutralMode(NeutralMode.Brake);
		//right3.setNeutralMode(NeutralMode.Brake);
		
	}
	
	public void disableBrake(){
		left1.setNeutralMode(NeutralMode.Coast);
		left2.setNeutralMode(NeutralMode.Coast);
	//	left3.setNeutralMode(NeutralMode.Coast);
		right1.setNeutralMode(NeutralMode.Coast);
		right2.setNeutralMode(NeutralMode.Coast);
		//right3.setNeutralMode(NeutralMode.Coast);
	}
	 
	public Encoder getRightEncoder(){
		return rightEncoder;
	}
	
	public Encoder getLeftEncoder(){
		return leftEncoder;
	}
	
	//Get Encoder Distances
	public double getRightEncoderDistance(){
		return rightEncoder.getDistance();
	}	
	public double getLeftEncoderDistance(){
		return leftEncoder.getDistance();
	}
	
	//Get Encoder counts
	public int getLeftEncoderCount(){
		return leftEncoder.get();
	}	
	public int getRightEncoderCount(){
		return rightEncoder.get() * -1;
	}
	
	//Get Encoder Rates
	public double getRightEncoderRate(){
		return rightEncoder.getRate();
	}	
	public double getLeftEncoderRate(){
		return leftEncoder.getRate();
	}
	
	//reset encoders
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
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