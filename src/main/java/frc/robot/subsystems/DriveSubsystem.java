/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends Subsystem {
	private TalonSRX left,right, leftScrew, rightScrew;
	private VictorSPX leftSlave, rightSlave;

	private double leftEncoderDist, rightEncoderDist, angle,leftScrewDist,rightScrewDist;
	private int leftEncoderCount, rightEncoderCount, leftScrewCount, rightScrewCount;

	private AHRS ahrs;
	private PowerDistributionPanel pdp;
	private boolean isStallCurrent = false;;
	private long startStallCurrent = 0;
	private double startBarometricPressure;
	private I2C bus;
	public DriveSubsystem() {
        System.out.println("Drive Subsystem Init");

		//Instantiate motors		
		left = new TalonSRX(RobotMap.LEFT_DRIVE_PORT_1);
		leftSlave = new VictorSPX(RobotMap.LEFT_DRIVE_PORT_2);
		right = new TalonSRX(RobotMap.RIGHT_DRIVE_PORT_1);		
		rightSlave = new VictorSPX(RobotMap.RIGHT_DRIVE_PORT_2);
		bus = new I2C(I2C.Port.kOnboard, 0x52);
		leftScrew = new TalonSRX(RobotMap.LEFT_SCREW_PORT);
		rightScrew = new TalonSRX(RobotMap.RIGHT_SCREW_PORT);
		


		
		//Instantiate Gyro | Gyro automatically calibrates when given power
        ahrs = new AHRS(SPI.Port.kMXP);
		
		pdp = new PowerDistributionPanel(11);
		startBarometricPressure = ahrs.getBarometricPressure();
		SmartDashboard.putData(ahrs);

	
	
		setupDrive();
		//setupScrew();	 
	} 
	public void setupDrive(){

		//Invert Motors
	
		right.setInverted(true);
		rightSlave.setInverted(true);
		
		//right3.setInverted(false);

		//Instantiate Encoders
		/* left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); */


		leftSlave.follow(left);
		rightSlave.follow(right);

		/* left.setSensorPhase(false);
		right.setSensorPhase(false);

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
 */


		left.configPeakCurrentLimit(60);
		left.configContinuousCurrentLimit(50);
		left.enableCurrentLimit(true);
		/* 
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
		right.configMotionAcceleration(ConstantsMap.DRIVE_ACCEL, ConstantsMap.kTimeoutMs); */

		right.configPeakCurrentLimit(60);
		right.configContinuousCurrentLimit(50);
		right.enableCurrentLimit(true);
	}
	public void setupScrew(){
		leftScrew.configFactoryDefault();
		leftScrew.setNeutralMode(NeutralMode.Brake);

		leftScrew.setInverted(false);
		leftScrew.overrideLimitSwitchesEnable(true);
		leftScrew.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				ConstantsMap.kPIDLoopIdx,
				ConstantsMap.kTimeoutMs);
		leftScrew.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		leftScrew.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		leftScrew.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
		leftScrew.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

		leftScrew.config_kF(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS.kF, ConstantsMap.kTimeoutMs);
		leftScrew.config_kP(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS.kP, ConstantsMap.kTimeoutMs);
		leftScrew.config_kI(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS.kI, ConstantsMap.kTimeoutMs);
		leftScrew.config_kD(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS.kD, ConstantsMap.kTimeoutMs);


		leftScrew.setSelectedSensorPosition(0);

		rightScrew.configFactoryDefault();
		rightScrew.setNeutralMode(NeutralMode.Brake);


		rightScrew.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				ConstantsMap.kPIDLoopIdx,
				ConstantsMap.kTimeoutMs);
		rightScrew.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		rightScrew.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		rightScrew.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
		rightScrew.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

		rightScrew.config_kF(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS2.kF, ConstantsMap.kTimeoutMs);
		rightScrew.config_kP(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS2.kP, ConstantsMap.kTimeoutMs);
		rightScrew.config_kI(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS2.kI, ConstantsMap.kTimeoutMs);
		rightScrew.config_kD(ConstantsMap.kPIDLoopIdx, ConstantsMap.SCREW_GAINS2.kD, ConstantsMap.kTimeoutMs);
	
		rightScrew.setSelectedSensorPosition(0);

	}
	
	public double getDistance(){
		//return bus.readOnly(buffer, count);
		return 0;
	}

	public boolean checkTip(){
		return (Math.abs(ahrs.getRoll()) > ConstantsMap.TIP_TOLERANCE) || (Math.abs(ahrs.getPitch()) > ConstantsMap.TIP_TOLERANCE);
		//return false;

	}
	public boolean checkDrift(){
		//return (Math.abs(ahrs.getRoll()) > ConstantsMap.DRIFT_TOLERANCE);
		return false;

	}    
	public double getVoltage(){
		return pdp.getVoltage();
		//return 12;
	}
	public boolean isStalled(){
		boolean isStallcondition = left.getOutputCurrent() > ConstantsMap.DRIVE_STALL || right.getOutputCurrent() > ConstantsMap.DRIVE_STALL;
		if(isStallcondition && !isStallCurrent){
			isStallCurrent = true;
			startStallCurrent = System.currentTimeMillis();
		}
		else if(!isStallcondition && isStallCurrent){
			isStallCurrent = false;
		}
		if((System.currentTimeMillis() - startStallCurrent) > ConstantsMap.DRIVE_STALL_TIME && isStallCurrent){
			return true;
		}
		else{
			return false;
		}
	}
	public boolean isCruisingAltitude(){
		//return ahrs.getAltitude() > 350000;
		return false;
	}

	public boolean isOnFire(){
		//return left.getTemperature() > 200 || right.getTemperature() > 200 || ahrs.getTempC() > 200;
		return false;

	}
	public boolean isProbablyAboutToRain(){
		//return (ahrs.getBarometricPressure() - startBarometricPressure) > 100;
		return false;

	}
	//sets the speed for both of the left motors
	public void setLeftSpeed(double speed) {
		left.set(ControlMode.PercentOutput,speed);
	}	
	
	//sets the speed for both of the right motors
	public void setRightSpeed (double speed) {
		right.set(ControlMode.PercentOutput,speed);
	}

	public void setLeftSpeedControl(double speed) {
		speed *= ConstantsMap.DRIVE_VELOCITY;
		left.set(ControlMode.Velocity,speed);
	}	
	
	//sets the speed for both of the right motors
	public void setRightSpeedControl (double speed) {
		speed *= ConstantsMap.DRIVE_VELOCITY;
		right.set(ControlMode.Velocity,speed);
	}
	
	/* public double getLeftSpeed() {		
		return leftEncoderRate * -1;
	}	
	
	public double getRightSpeed() {		
		return rightEncoderRate * -1;	
	} */
	
	public void tankDrive(double leftSpeed, double rightSpeed){
		setLeftSpeed(leftSpeed);
		setRightSpeed(rightSpeed);
	}
	
	public void enableBrake(){
		//enable on left
		left.setNeutralMode(NeutralMode.Brake);

		//enable on right
		right.setNeutralMode(NeutralMode.Brake);
	}
	
	public void disableBrake(){
		//disable on left
		left.setNeutralMode(NeutralMode.Coast);
		
		//disable on right
		right.setNeutralMode(NeutralMode.Coast);
	}
	 
	
	
	//Get Encoder Distances
	public double getRightEncoderDistance(){
		return rightEncoderDist;
	}	
	public double getLeftEncoderDistance(){
		return leftEncoderDist;
	}
	
	//Get Encoder counts
	public int getLeftEncoderCount(){
		return leftEncoderCount;
	}	
	public int getRightEncoderCount(){
		return rightEncoderCount;
	}
	
	/* //Get Encoder Rates
	public double getRightEncoderRate(){
		return rightEncoderRate;
	}	
	public double getLeftEncoderRate(){
		return leftEncoderRate;
	}
	 */
	//reset encoders
	public void resetEncoders(){
		left.getSensorCollection().setQuadraturePosition(0, 0);
		right.getSensorCollection().setQuadraturePosition(0, 0);
	}
	
	public AHRS getGyro(){
		return ahrs;
	}
	
	public double getGyroAngle(){
		return angle;
	}

	public void resetGyro() {
		ahrs.reset();
	}

	public void stop() {
		left.set(ControlMode.PercentOutput,0);
		right.set(ControlMode.PercentOutput,0);
		
	}

	//HAB STUFF

	public void setScrewVelocitySetpoint(double sp) {
		//leftScrew.set(ControlMode.Velocity,sp);
		rightScrew.set(ControlMode.Velocity,sp);
	}

	public void setScrewSpeed(double sp) {
		leftScrew.set(ControlMode.PercentOutput,sp);
		rightScrew.set(ControlMode.PercentOutput,sp);
	}

	public void stopScrew() {
		System.out.println("Stop Screw");
		leftScrew.set(ControlMode.PercentOutput,0);
		rightScrew.set(ControlMode.PercentOutput,0);
	}

	/**
	 * @return the rightScrewDist
	 */
	public double getRightScrewDist() {
		return rightScrewDist;
	}
	/**
	 * @return the leftScrewDist
	 */
	public double getLeftScrewDist() {
		return leftScrewDist;
	}
	/**
	 * @return the leftScrewCount
	 */
	public int getLeftScrewCount() {
		return leftScrewCount;
	}
	/**
	 * @return the rightScrewCount
	 */
	public int getRightScrewCount() {
		return rightScrewCount;
	}

	public void checkSlave(){
		if(Math.abs(leftScrewCount-rightScrewCount)>ConstantsMap.SCREW_TOLERANCE){
			leftScrew.set(ControlMode.Position, rightScrewCount);
		}
	}
	@Override
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveCommand());
	}
	
	@Override
	public void periodic() {

		angle = ahrs.getAngle();
		/* rightEncoderCount = right.getSelectedSensorPosition(0);
		leftEncoderCount = left.getSelectedSensorPosition(0);
		rightEncoderDist = rightEncoderCount * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;
		leftEncoderDist = leftEncoderCount * ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK;

		rightScrewCount = rightScrew.getSelectedSensorPosition(0);
		leftScrewCount = leftScrew.getSelectedSensorPosition(0);
		rightScrewDist = (double)rightScrewCount * ConstantsMap.SCREW_INCHES_PER_TICK;
		leftScrewDist = (double)leftScrewCount * ConstantsMap.SCREW_INCHES_PER_TICK;

		checkSlave(); */

		
	}
}