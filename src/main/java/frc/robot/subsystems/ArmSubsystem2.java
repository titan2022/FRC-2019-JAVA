/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ConstantsMap;
import frc.robot.RobotMap;
import frc.robot.commands.ArmCommand2;

/**
 * Add your docs here.
 */
public class ArmSubsystem2 extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double wristAngle, wristSpeed, leftShoulderAngle, rightShoulderAngle, shoulderSpeed;
    
    public int wristCount, oldWristCount, leftShoulderCount, rightShoulderCount;

    private DigitalInput lowerLimit, lowerLimitWrist, lowerLimitWrist2;
    private double wristLimitStartTime,wristLimitStartTime2;
    private boolean wristLimit,wristLimit2;


    private double shoulderSet;
    private double wristSet;
    private boolean manualMode;
    private boolean levelMode;

    private boolean isWristStall = false;
    private boolean isShoulderStall = false;
    private long startWristStall = 0;
    private long startShoulderStall = 0;
    TalonSRX shoulder, shoulder2, shoulderSlave, wrist, wristSlave, wrist2;

    public ArmSubsystem2() {

        lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        lowerLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimitWrist2 = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT2);


        shoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_RIGHT_PORT);
        //shoulderSlave = new TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);
        //shoulderSlave.configFactoryDefault();
        shoulder2 = new TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);

        
        shoulder.setInverted(true);
        shoulder2.setInverted(true);

        shoulder.setSensorPhase(false);
        shoulder2.setSensorPhase(true);

        shoulder.setNeutralMode(NeutralMode.Brake);
        //shoulderSlave.setNeutralMode(NeutralMode.Brake);

        //shoulderSlave.follow(shoulder);
        
        shoulder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
											ConstantsMap.kPIDLoopIdx, 
											ConstantsMap.kTimeoutMs);
        shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ConstantsMap.kTimeoutMs);
		shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsMap.kTimeoutMs);

        shoulder.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		shoulder.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		shoulder.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
        shoulder.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

        shoulder.selectProfileSlot(ConstantsMap.kSlotIdx, ConstantsMap.kPIDLoopIdx);
		shoulder.config_kF(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kF, ConstantsMap.kTimeoutMs);
		shoulder.config_kP(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kP, ConstantsMap.kTimeoutMs);
		shoulder.config_kI(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kI, ConstantsMap.kTimeoutMs);
		shoulder.config_kD(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kD, ConstantsMap.kTimeoutMs);
   
        shoulder.configMotionCruiseVelocity(ConstantsMap.SHOULDER_VELOCITY, ConstantsMap.kTimeoutMs);
        shoulder.configMotionAcceleration(ConstantsMap.SHOULDER_ACCEL, ConstantsMap.kTimeoutMs);
        

        shoulder.configPeakCurrentLimit(40);
		shoulder.configContinuousCurrentLimit(30);
		shoulder.enableCurrentLimit(true);


        shoulder2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
											ConstantsMap.kPIDLoopIdx, 
											ConstantsMap.kTimeoutMs);
        shoulder2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ConstantsMap.kTimeoutMs);
		shoulder2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsMap.kTimeoutMs);

        shoulder2.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		shoulder2.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		shoulder2.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
        shoulder2.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

        shoulder2.selectProfileSlot(ConstantsMap.kSlotIdx, ConstantsMap.kPIDLoopIdx);
		shoulder2.config_kF(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kF, ConstantsMap.kTimeoutMs);
		shoulder2.config_kP(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kP, ConstantsMap.kTimeoutMs);
		shoulder2.config_kI(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kI, ConstantsMap.kTimeoutMs);
		shoulder2.config_kD(ConstantsMap.kSlotIdx, ConstantsMap.shoulderGains.kD, ConstantsMap.kTimeoutMs);
   
        shoulder2.configMotionCruiseVelocity(ConstantsMap.SHOULDER_VELOCITY, ConstantsMap.kTimeoutMs);
        shoulder2.configMotionAcceleration(ConstantsMap.SHOULDER_ACCEL, ConstantsMap.kTimeoutMs);


        

        shoulder2.configPeakCurrentLimit(40);
		shoulder2.configContinuousCurrentLimit(30);
		shoulder2.enableCurrentLimit(true);
        //shoulder.configForwardLimitSwitchSource(LimitSwitchSource., LimitSwitchNormal.NormallyClosed);
        wrist = new TalonSRX(RobotMap.WRIST_JOINT_LEFT_PORT);
        wrist2 = new TalonSRX(RobotMap.WRIST_JOINT_RIGHT_PORT);

        wrist.setInverted(false);
        wrist2.setInverted(true);

        wrist.setSensorPhase(false);
        //wrist2.setSensorPhase(true);

        wrist.setNeutralMode(NeutralMode.Brake);
        wrist2.setNeutralMode(NeutralMode.Brake);
        wrist2.follow(wrist);

        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
											ConstantsMap.kPIDLoopIdx, 
                                            ConstantsMap.kTimeoutMs);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ConstantsMap.kTimeoutMs);
		wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ConstantsMap.kTimeoutMs);

        wrist.configNominalOutputForward(0, ConstantsMap.kTimeoutMs);
		wrist.configNominalOutputReverse(0, ConstantsMap.kTimeoutMs);
		wrist.configPeakOutputForward(1, ConstantsMap.kTimeoutMs);
        wrist.configPeakOutputReverse(-1, ConstantsMap.kTimeoutMs);

        wrist.selectProfileSlot(ConstantsMap.kSlotIdx, ConstantsMap.kPIDLoopIdx);
		wrist.config_kF(ConstantsMap.kSlotIdx, ConstantsMap.wristGains.kF, ConstantsMap.kTimeoutMs);
		wrist.config_kP(ConstantsMap.kSlotIdx, ConstantsMap.wristGains.kP, ConstantsMap.kTimeoutMs);
		wrist.config_kI(ConstantsMap.kSlotIdx, ConstantsMap.wristGains.kI, ConstantsMap.kTimeoutMs);
		wrist.config_kD(ConstantsMap.kSlotIdx, ConstantsMap.wristGains.kD, ConstantsMap.kTimeoutMs);
   
        wrist.configMotionCruiseVelocity(ConstantsMap.WRIST_VELOCITY, ConstantsMap.kTimeoutMs);
        wrist.configMotionAcceleration(ConstantsMap.WRIST_ACCEL, ConstantsMap.kTimeoutMs);     


        lowerLimit.setName("LowerWristLimit");
        oldWristCount = wrist.getSelectedSensorPosition();
        periodic();

        setWristSetPoint(getWristEncoderAngle());
        setShoulderSetPoint(getShoulderEncoderAngle());

        manualMode = false;
        levelMode = false;
    }
    
    //Modes
    public boolean getManualMode(){
        return manualMode;
    }
    public boolean getLevelMode(){
        return levelMode;
    }
    public void toggleManualMode(){
        manualMode =  !manualMode;
    }
    public void toggleLevelMode(){
        levelMode =  !levelMode;
    }

    //Wrist Stuff
    
    public void setWristJointSpeed(double speed) {
        wrist.set(ControlMode.PercentOutput,speed);
        //wrist2.set(ControlMode.PercentOutput, speed);
    }
    
    public void setWristSetPoint(double angle){
        if(shoulder.getControlMode() == ControlMode.MotionMagic){
            
            if(shoulderSet*ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK<ConstantsMap.SHOULDER_WRIST_FOLD_ANGLE){
                angle = ConstantsMap.WRIST_MAX_ANGLE;

            }
            else {
                
                if(angle>ConstantsMap.WRIST_MAX_ANGLE){
                    angle =ConstantsMap.WRIST_MAX_ANGLE;
                }
                if(angle<ConstantsMap.WRIST_MIN_ANGLE_UP){
                    angle =ConstantsMap.WRIST_MIN_ANGLE_UP;
                }
            }
            
        }
        else{
            if(getShoulderEncoderAngle()<ConstantsMap.SHOULDER_WRIST_FOLD_ANGLE){
                angle = ConstantsMap.WRIST_MAX_ANGLE;
            }
            if(angle>ConstantsMap.WRIST_MAX_ANGLE){
                angle =ConstantsMap.WRIST_MAX_ANGLE;
            }
            if(angle<ConstantsMap.WRIST_MIN_ANGLE_UP){
                angle =ConstantsMap.WRIST_MIN_ANGLE_UP;
            }
        }
        
       
        //System.out.println(shoulderSet + " : " + angle);

        double ticks = angle/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;
        
        wrist.set(ControlMode.MotionMagic, ticks);
        //wrist2.set(ControlMode.MotionMagic, ticks);
        wristSet = ticks;
    }
   

    public void checkWristLimits(){
        
        if(getWristLowerLimit()){
           // if(getShoulderEncoderAngle()<-50){
               int zero = (int)(ConstantsMap.WRIST_MAX_ANGLE/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK);
                wrist.setSelectedSensorPosition(zero);
                oldWristCount = zero;
                //wrist2.setSelectedSensorPosition((int)(ConstantsMap.WRIST_MAX_ANGLE/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK));

            //}
            if(wrist.getControlMode() == ControlMode.PercentOutput){
                if(wrist.getMotorOutputPercent()>0){
                    wrist.set(ControlMode.PercentOutput, 0);
                    //wrist2.set(ControlMode.PercentOutput, 0);

                }
            }
            else if (wrist.getControlMode() == ControlMode.MotionMagic){
                if(wrist.getActiveTrajectoryVelocity() > 0){
                
                    setWristSetPoint(ConstantsMap.WRIST_MAX_ANGLE);
                }
            }
        }
        else if(Math.abs(oldWristCount-wristCount)>(90.0/ConstantsMap.DRIVE_ENCODER_DIST_PER_TICK)){
            wrist.setSelectedSensorPosition(oldWristCount);

        }
        if(getWristEncoderAngle()<ConstantsMap.WRIST_MIN_ANGLE_UP){
            if(wrist.getActiveTrajectoryVelocity() < 0){
                
                setWristJointSpeed(0);
            }
        }
    }

    public double getWristSetPoint(){
        return wrist.getActiveTrajectoryPosition()* ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;

    }

    public double getWristEncoderAngle() {
        return wristAngle;
    }

    public double getWristTicks(){
        return wristCount;
    }

    public void zeroWrist() {
        wrist.setSelectedSensorPosition((int)(ConstantsMap.WRIST_MAX_ANGLE/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK), ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);
        //wrist2.setSelectedSensorPosition(0, ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);

    }

    public boolean getWristLowerLimit(){
        double currentTime = System.currentTimeMillis();

        if(lowerLimitWrist.get()){

            if(!wristLimit){
                wristLimitStartTime = System.currentTimeMillis();
                wristLimit = true;
            }   
            if(currentTime - wristLimitStartTime > 100){
                wristLimit = true;
            }
        }
        else{
            wristLimit = false;

        }
        if(lowerLimitWrist2.get()){
            if(!wristLimit2){
                wristLimitStartTime2 = System.currentTimeMillis();
            
            }  
            if(currentTime - wristLimitStartTime2 > 100){
                wristLimit2 = true;
            }
        }
        else{
            wristLimit2 = false;

        }
        
       // return wristLimit || wristLimit2;
       return lowerLimitWrist2.get() || lowerLimitWrist.get();
    }

    public boolean isWristStalled(){
		boolean isStallcondition = wrist.getOutputCurrent() > ConstantsMap.WRIST_STALL;
		if(isStallcondition && !isWristStall){
			isWristStall = true;
			startWristStall = System.currentTimeMillis();
		}
		else if(!isStallcondition && isWristStall){
			isWristStall = false;
		}
		if((System.currentTimeMillis() - startWristStall) > ConstantsMap.WRIST_STALL_TIME && isWristStall){
			return true;
		}
		else{
			return false;
		}
    }

    public double getWristSpeed(){
        return wristSpeed;
    }
    public boolean isWristAtSetPoint(){
        return Math.abs(getWristSetPoint()-getWristEncoderAngle()) < ConstantsMap.WRIST_TOLERANCE;
    }
    public boolean isWristAtSetPoint(double setPoint){
        return Math.abs(setPoint-getWristEncoderAngle()) < ConstantsMap.WRIST_TOLERANCE;
    }

    //Shoulder Stuff

    public void setShoulderJointSpeed(double speed) {
        
        shoulder.set(ControlMode.PercentOutput,speed);
        shoulder2.set(ControlMode.PercentOutput,speed);

    }
    public void setShoulderSetPoint(double angle){
        if(angle>ConstantsMap.SHOULDER_MAX_ANGLE){
            angle =ConstantsMap.SHOULDER_MAX_ANGLE;
        }
        if(angle<ConstantsMap.SHOULDER_MIN_ANGLE){
            angle =ConstantsMap.SHOULDER_MIN_ANGLE;
        }
        double ticks = angle/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
        shoulder.set(ControlMode.MotionMagic, ticks);
        shoulder2.set(ControlMode.MotionMagic, ticks);

        shoulderSet = ticks;

    }

    public void checkShoulderLimits(){
        if(!lowerLimit.get()){
            if(getShoulderEncoderAngle()<ConstantsMap.SHOULDER_MIN_ANGLE + 30){
                shoulder2.setSelectedSensorPosition((int)(ConstantsMap.SHOULDER_MIN_ANGLE/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK));
                shoulder.setSelectedSensorPosition((int)(ConstantsMap.SHOULDER_MIN_ANGLE/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK));

            }

            if(shoulder.getControlMode() == ControlMode.PercentOutput){
                if(shoulder.getMotorOutputPercent()<0){
                    shoulder.set(ControlMode.PercentOutput, 0);
                    shoulder2.set(ControlMode.PercentOutput, 0);

                }
            }
            else if (shoulder.getControlMode() == ControlMode.MotionMagic){
                if(shoulder.getActiveTrajectoryVelocity() < 0){
                    setShoulderSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);
                }
            }         
            
        }
   }
    
    public double getRightShoulderTicks(){
        return rightShoulderCount;
    }
    public double getLeftShoulderTicks(){
        return leftShoulderCount;
    }
   

    public double getRightShoulderEncoderAngle() {
        return rightShoulderAngle;
    }
    public double getLeftShoulderEncoderAngle() {
        return leftShoulderAngle;
    }    
    public double getShoulderEncoderAngle(){
        return (getRightShoulderEncoderAngle() + getLeftShoulderEncoderAngle())/2;
    }
    public double getRightShoulderSetPoint(){
        return shoulder.getActiveTrajectoryPosition()* ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
    }
    public double getLeftShoulderSetPoint(){
        return shoulder2.getActiveTrajectoryPosition()* ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
    }

    
    public void zeroShoulder() {
        shoulder.setSelectedSensorPosition((int)(ConstantsMap.SHOULDER_MIN_ANGLE/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK), ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);
        shoulder2.setSelectedSensorPosition((int)(ConstantsMap.SHOULDER_MIN_ANGLE/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK), ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);

    }    

    public boolean getShoulderLowerLimit(){
        return !lowerLimit.get();
    }



   

    public double getShoulderSpeed(){
        return shoulder.getMotorOutputPercent();
    }

    public boolean isShoulderAtSetPoint(double setPoint){
        boolean right = Math.abs(setPoint-getRightShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        boolean left = Math.abs(setPoint-getLeftShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        return left && right;

    }
    public boolean isShoulderAtSetPoint(){
        boolean right = Math.abs(getRightShoulderSetPoint()-getRightShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        boolean left = Math.abs(getLeftShoulderSetPoint()-getLeftShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        return left && right;

    }
    public boolean isLeftShoulderAtSetPoint(){
       // boolean right = Math.abs(getRightShoulderSetPoint()-getRightShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        boolean left = Math.abs(getLeftShoulderSetPoint()-getLeftShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        return left;

    }
    public boolean isRightShoulderAtSetPoint(){
        boolean right = Math.abs(getRightShoulderSetPoint()-getRightShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        //boolean left = Math.abs(getLeftShoulderSetPoint()-getLeftShoulderEncoderAngle()) < ConstantsMap.SHOULDER_TOLERANCE;
        return right;

    }
    
    public boolean isShoulderStalled(){
		boolean isStallcondition = shoulder.getOutputCurrent() > ConstantsMap.SHOULDER_STALL;
		if(isStallcondition && !isShoulderStall){
			isShoulderStall = true;
			startShoulderStall = System.currentTimeMillis();
		}
		else if(!isStallcondition && isShoulderStall){
			isShoulderStall = false;
		}
		if((System.currentTimeMillis() - startShoulderStall) > ConstantsMap.SHOULDER_STALL_TIME && isShoulderStall){
			return true;
		}
		else{
			return false;
		}
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
         setDefaultCommand(new ArmCommand2());
    }

    @Override
    public void periodic() {
        wristCount = wrist.getSelectedSensorPosition(0);
        leftShoulderCount = shoulder2.getSelectedSensorPosition(0);
        rightShoulderCount = shoulder.getSelectedSensorPosition(0);
        wristAngle = wristCount * ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;
        leftShoulderAngle = leftShoulderCount * ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
        rightShoulderAngle = rightShoulderCount * ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
        shoulderSpeed = shoulder.getMotorOutputPercent();
        wristSpeed = wrist.getMotorOutputPercent(); 
        checkWristLimits();
        checkShoulderLimits();
        oldWristCount = wristCount;

        
    }
}
