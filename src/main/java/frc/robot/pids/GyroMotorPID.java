package frc.robot.pids;

import frc.robot.TalonSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class GyroMotorPID implements PIDSource, PIDOutput {
    TalonSet motor;
    PIDController pid;
    PIDSourceType pidSourceType;
    PIDSource source;
    AHRS ahrs;

    public GyroMotorPID(TalonSet mot, AHRS gyro, double kp, double ki, double kd, double kf) {
        motor = mot;
        pid = new PIDController(kp, ki, kd, kf, this, this);
        pidSourceType = PIDSourceType.kDisplacement;
        ahrs = gyro;
    }
    public PIDController getPID(){
        return this.pid;
    }
    public GyroMotorPID setP(double kp) {
        pid.setP(kp);
        return this;
    }

    public GyroMotorPID setI(double ki) {
        pid.setI(ki);
        return this;
    }

    public GyroMotorPID setD(double kd) {
        pid.setD(kd);
        return this;
    }

    /* public EncoderMotorPID setDistancePerTick(double dpt) {
        encoder.setDistancePerPulse(dpt);
        return this;
    } */

    public GyroMotorPID setOutputRange(double l, double h) {
        pid.setOutputRange(l,h);
        return this;
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public void pidWrite(double speed) {
        motor.set(speed);
    }

    public void setPIDSourceType(PIDSourceType t) {
        pidSourceType = t;
    }

    public PIDSourceType getPIDSourceType() {
        return ahrs.getPIDSourceType();
    }

    public double pidGet() {
        return ahrs.pidGet();
    }

    public GyroMotorPID setSetpoint(double sp) {
        pid.setSetpoint(sp);
        return this;
    }

    public boolean isEnabled() {
        return pid.isEnabled();
    }

    public void enable() {
        pid.enable();
    }

    public void disable() {
        pid.disable();
    }
}
