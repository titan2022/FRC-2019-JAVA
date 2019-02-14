package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/*
 * Controls a group of motors as a unit.
 */
public class TalonSet {
    TalonSRX motors[];
    double dpt;

    public TalonSet(TalonSRX[] talons) {
        motors = talons;
        for (TalonSRX motor : motors) motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    }

    public TalonSet(TalonSRX talon) {
        motors = new TalonSRX[] { talon };
    }

    public TalonSet setDistance(double dpt) {
        this.dpt = dpt;
        return this;
    }

    public void set(double speed) {
        for (TalonSRX motor : motors) motor.set(ControlMode.PercentOutput,speed);
    }

    public double get() {
        // As all members of a TalonSet should move in unison,
        // if this value is incorrect, you have a bigger problem
        // on your hands.
        return motors[0].getMotorOutputPercent();
    }

    public int getEncoderTicks() {
        return motors[0].getSelectedSensorPosition(0);
    }

    public double getEncoderDistance() {
        return getEncoderTicks() * dpt;
    }
}
