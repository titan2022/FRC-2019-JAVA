package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*
 * Controls a group of motors as a unit.
 */
public class TalonSet {
    WPI_TalonSRX motors[];

    public TalonSet(WPI_TalonSRX[] talons) {
        motors = talons;
    }

    public TalonSet(WPI_TalonSRX talon) {
        motors = new WPI_TalonSRX[] { talon };
    }

    public void set(double speed) {
        for (WPI_TalonSRX motor : motors) motor.set(speed);
    }

    public double get() {
        // As all members of a TalonSet should move in unison,
        // if this value is incorrect, you have a bigger problem
        // on your hands.
        return motors[0].get();
    }
}
