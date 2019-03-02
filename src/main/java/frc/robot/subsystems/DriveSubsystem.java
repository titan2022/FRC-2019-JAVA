package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import frc.robot.ConstantsMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 */
public class DriveSubsystem extends Subsystem {

	private TalonSRX left,leftSlave,right,rightSlave;

	private AHRS ahrs;
	

	public DriveSubsystem() {
        System.out.println("Drive Subsystem Init");

		//Instantiate motors		
		left = new TalonSRX(RobotMap.LEFT_DRIVE_PORT_2);
		leftSlave = new TalonSRX(RobotMap.LEFT_DRIVE_PORT_1);
		right = new TalonSRX(RobotMap.RIGHT_DRIVE_PORT_1);		
		rightSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_PORT_2);
		
		//Invert Motors
		
		right.setInverted(true);
		rightSlave.setInverted(true);
		//right3.setInverted(false);
		
		//Instantiate Encoders
		left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		
		//Instantiate Gyro | Gyro automatically calibrates when given power
        ahrs = new AHRS(SPI.Port.kMXP);
		stop();
		

		
		SmartDashboard.putData(ahrs);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveCommand());
    }
	
	public void setTravel(double distance){		
		left.set(ControlMode.MotionMagic, getLeftEncoderDistance() + distance);
		right.set(ControlMode.MotionMagic, getRightEncoderDistance() + distance);
	}
	//sets the speed for both of the left motors
	public void setLeftSpeed(double speed) {
		left.set(ControlMode.PercentOutput,speed);
	}	
	
	//sets the speed for both of the right motors
	public void setRightSpeed (double speed) {
		right.set(ControlMode.PercentOutput,speed);
	}
	
	public double getLeftSpeed() {		
		return left.getSelectedSensorVelocity(0);
	}	
	
	public double getRightSpeed() {		
		return right.getSelectedSensorVelocity(0);		
	}
	
	public void tankDrive(double leftSpeed, double rightSpeed){
		setLeftSpeed(leftSpeed);
		setRightSpeed(rightSpeed);
	}
	
	public void enableBrake(){
		//enable on left
		left.setNeutralMode(NeutralMode.Brake);
		leftSlave.setNeutralMode(NeutralMode.Brake);

		//enable on right
		right.setNeutralMode(NeutralMode.Brake);
		rightSlave.setNeutralMode(NeutralMode.Brake);
	}
	
	public void disableBrake(){
		//disable on left
		left.setNeutralMode(NeutralMode.Coast);
		leftSlave.setNeutralMode(NeutralMode.Coast);
		
		//disable on right
		right.setNeutralMode(NeutralMode.Coast);
		rightSlave.setNeutralMode(NeutralMode.Coast);
	}
	 

	//Get Encoder Distances
	public double getRightEncoderDistance(){
		return right.getSelectedSensorPosition(0) * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
	}	
	public double getLeftEncoderDistance(){
		return left.getSelectedSensorPosition(0) * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
	}
	
	//Get Encoder counts
	public int getLeftEncoderCount(){
		return left.getSelectedSensorPosition(0);
	}	
	public int getRightEncoderCount(){
		return right.getSelectedSensorPosition(0);
	}
	
	//Get Encoder Rates
	public double getRightEncoderRate(){
		return right.getSelectedSensorVelocity(0);
	}	
	public double getLeftEncoderRate(){
		return left.getSelectedSensorVelocity(0);
	}
	
	//reset encoders
	public void resetEncoders(){
		left.getSensorCollection().setQuadraturePosition(0, 0);
		right.getSensorCollection().setQuadraturePosition(0, 0);
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
		left.set(ControlMode.PercentOutput,0);
		right.set(ControlMode.PercentOutput,0);
	}
    
}