package frc.robot.pids;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// TODO: change the goddamn name
/*
 * Controls a group of motors as a unit.
 */
public class PIDMultiOutput {
    WPI_TalonSRX motors[];

    public PIDMultiOutput(WPI_TalonSRX[] talons) {
        motors = talons;
    }

    public PIDMultiOutput(WPI_TalonSRX talon) {
        motors = new WPI_TalonSRX[] { talon };
    }

    public void set(double speed) {
        for (WPI_TalonSRX motor : motors)
            motor.set(speed);
    }
}
