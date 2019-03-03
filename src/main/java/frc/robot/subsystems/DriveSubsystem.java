package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import frc.robot.ConstantsMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
		

		left.setSensorPhase(true);
		right.setSensorPhase(true);

		left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											ConstantsMap.kPIDLoopIdx, 
											ConstantsMap.kTimeoutMs);
        left.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ConstantsMap.kTimeoutMs);
		left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsMap.kTimeoutMs);

        left.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		left.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		left.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
        left.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

        left.selectProfileSlot(ConstantsMap.kSlotIdx, ConstantsMap.kPIDLoopIdx);
		left.config_kF(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kF, ConstantsMap.kTimeoutMs);
		left.config_kP(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kP, ConstantsMap.kTimeoutMs);
		left.config_kI(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kI, ConstantsMap.kTimeoutMs);
		left.config_kD(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kD, ConstantsMap.kTimeoutMs);
   
        left.configMotionCruiseVelocity(ConstantsMap.DRIVE_VELOCITY, ConstantsMap.kTimeoutMs);
        left.configMotionAcceleration(ConstantsMap.DRIVE_ACCEL, ConstantsMap.kTimeoutMs);


		

		right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											ConstantsMap.kPIDLoopIdx, 
											ConstantsMap.kTimeoutMs);
        right.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ConstantsMap.kTimeoutMs);
		right.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsMap.kTimeoutMs);

        right.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		right.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		right.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
        right.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

        right.selectProfileSlot(ConstantsMap.kSlotIdx, ConstantsMap.kPIDLoopIdx);
		right.config_kF(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kF, ConstantsMap.kTimeoutMs);
		right.config_kP(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kP, ConstantsMap.kTimeoutMs);
		right.config_kI(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kI, ConstantsMap.kTimeoutMs);
		right.config_kD(ConstantsMap.kSlotIdx, ConstantsMap.driveGains.kD, ConstantsMap.kTimeoutMs);
   
        right.configMotionCruiseVelocity(ConstantsMap.DRIVE_VELOCITY, ConstantsMap.kTimeoutMs);
        right.configMotionAcceleration(ConstantsMap.DRIVE_ACCEL, ConstantsMap.kTimeoutMs);




		enableBrake();




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
		resetEncoders();
		left.set(ControlMode.MotionMagic, distance);
		right.set(ControlMode.MotionMagic, distance);
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