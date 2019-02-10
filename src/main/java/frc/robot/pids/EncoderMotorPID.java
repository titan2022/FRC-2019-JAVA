package frc.robot.pids;

import frc.robot.TalonSet;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderMotorPID implements PIDSource, PIDOutput {
    Encoder encoder;
    TalonSet motor;
    PIDController pid;
    PIDSourceType pidSourceType;
    double dpt;

    public EncoderMotorPID(Encoder enc, TalonSet mot, double kp, double ki, double kd, double kf, double dpt) {
        encoder = enc;
        motor = mot;
        pid = new PIDController(kp, ki, kd, kf, this, this);
        pidSourceType = PIDSourceType.kDisplacement;
        this.dpt = dpt;
    }

    public EncoderMotorPID setP(double kp) {
        pid.setP(kp);
        return this;
    }

    public EncoderMotorPID setI(double ki) {
        pid.setI(ki);
        return this;
    }

    public EncoderMotorPID setD(double kd) {
        pid.setD(kd);
        return this;
    }

    public EncoderMotorPID setDistancePerTick(double dpt) {
        encoder.setDistancePerPulse(dpt);
        return this;
    }

    public EncoderMotorPID setOutputRange(double l, double h) {
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
        return pidSourceType;
    }

    public double pidGet() {
        return motor.encTicks() * dpt;
    }

    public EncoderMotorPID setSetpoint(double sp) {
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
