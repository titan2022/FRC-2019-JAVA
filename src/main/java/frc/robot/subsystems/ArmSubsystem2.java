/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsMap;
import frc.robot.RobotMap;
import frc.robot.TalonSet;
import frc.robot.commands.ArmCommand2;

/**
 * Add your docs here.
 */
public class ArmSubsystem2 extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.



    private DigitalInput lowerLimit, lowerLimitWrist, lowerLimitWrist2;
    private double wristLimitStartTime,wristLimitStartTime2;
    private boolean wristLimit,wristLimit2;
    TalonSRX shoulder, shoulderSlave,wrist;

    public ArmSubsystem2() {

        lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        lowerLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimitWrist2 = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT2);


        shoulder = new TalonSRX(RobotMap.SHOULDER_JOINT_RIGHT_PORT);
        shoulderSlave = new TalonSRX(RobotMap.SHOULDER_JOINT_LEFT_PORT);
        shoulderSlave.configFactoryDefault();
        shoulder.setInverted(false);
        shoulderSlave.setInverted(true);
        shoulder.setSensorPhase(true);
        shoulder.setNeutralMode(NeutralMode.Brake);
        shoulderSlave.follow(shoulder);
        
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
        
        //shoulder.configForwardLimitSwitchSource(LimitSwitchSource., LimitSwitchNormal.NormallyClosed);
        wrist = new TalonSRX(RobotMap.WRIST_JOINT_PORT);
        wrist.setInverted(true);
        wrist.setNeutralMode(NeutralMode.Brake);
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
											ConstantsMap.kPIDLoopIdx, 
                                            ConstantsMap.kTimeoutMs);
        wrist.setSensorPhase(true);
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

        //upperLimit = new DigitalInput(RobotMap.UPPER_ARM_LIMIT_PORT);
        //lowerLimit = new DigitalInput(RobotMap.LOWER_ARM_LIMIT_PORT);
        //upperLimitWrist = new DigitalInput(RobotMap.LOWER_WRIST_LIMIT_PORT);
        lowerLimit.setName("LowerWristLimit");

        setWristSetPoint(getWristEncoderAngle());
        setShoulderSetPoint(getShoulderEncoderAngle());
        
    }
    
    public void setShoulderJointSpeed(double speed) {
        
        shoulder.set(ControlMode.PercentOutput,speed);
    }

    public void setWristJointSpeed(double speed) {
        wrist.set(ControlMode.PercentOutput,speed);
    }
    public void setWristSetPoint(double angle){
        if(getShoulderEncoderAngle()<ConstantsMap.SHOULDER_WRIST_FOLD_ANGLE){
            angle = ConstantsMap.WRIST_MAX_ANGLE;
        }
        if(angle>ConstantsMap.WRIST_MAX_ANGLE){
            angle =ConstantsMap.WRIST_MAX_ANGLE;
        }
        if(angle<ConstantsMap.WRIST_MIN_ANGLE){
            angle =ConstantsMap.WRIST_MIN_ANGLE;
        }
        
        double ticks = angle/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;
        
        wrist.set(ControlMode.MotionMagic, ticks);
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
        checkShoulderLimits();

    }

    public void checkWristLimits(){
        if(getWristLowerLimit()){
            wrist.setSelectedSensorPosition((int)(ConstantsMap.WRIST_MAX_ANGLE/ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK));
            if(wrist.getControlMode() == ControlMode.PercentOutput){
                if(wrist.getMotorOutputPercent()>0){
                    wrist.set(ControlMode.PercentOutput, 0);
                }
            }
            else if (wrist.getControlMode() == ControlMode.MotionMagic){
                if(wrist.getActiveTrajectoryVelocity() > 0){
                    setWristSetPoint(ConstantsMap.WRIST_MAX_ANGLE);
                }
            }
        }
    }
    public void checkShoulderLimits(){
        if(!lowerLimit.get()){
            shoulder.setSelectedSensorPosition((int)(ConstantsMap.SHOULDER_MIN_ANGLE/ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK));
            if(shoulder.getControlMode() == ControlMode.PercentOutput){
                if(shoulder.getMotorOutputPercent()<0){
                    shoulder.set(ControlMode.PercentOutput, 0);
                }
            }
            else if (shoulder.getControlMode() == ControlMode.MotionMagic){
                if(shoulder.getActiveTrajectoryVelocity() < 0){
                    setShoulderSetPoint(ConstantsMap.SHOULDER_MIN_ANGLE);
                }
            }
            
            
        }
   }


    public double getShoulderEncoderAngle() {
        return getShoulderTicks()* ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
    }
    public double getShoulderTicks(){
        return shoulder.getSelectedSensorPosition();
    }
    public double getShoulderSetPoint(){
        return shoulder.getActiveTrajectoryPosition()* ConstantsMap.SHOULDER_ENCODER_ANGLE_PER_TICK;
    }
    public double getWristSetPoint(){
        return wrist.getActiveTrajectoryPosition()* ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;

    }

    public double getWristEncoderAngle() {
        return getWristTicks()* ConstantsMap.WRIST_ENCODER_ANGLE_PER_TICK;
    }
    public double getWristTicks(){
        return wrist.getSelectedSensorPosition();
    }
    public void zeroWrist() {
        wrist.setSelectedSensorPosition(0, ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);
    }
    public void zeroShoulder() {
        shoulder.setSelectedSensorPosition(0, ConstantsMap.kPIDLoopIdx, ConstantsMap.kTimeoutMs);
    }


    public boolean getShoulderLowerLimit(){
        return !lowerLimit.get();
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

    public double getShoulderSpeed(){
        return shoulder.getMotorOutputPercent();
    }

    public double getWristSpeed(){
        return wrist.getMotorOutputPercent();
    }
    
    
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
         setDefaultCommand(new ArmCommand2());
    }
}
