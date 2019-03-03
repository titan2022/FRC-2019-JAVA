package frc.robot.pids;

import frc.robot.Robot;
import frc.robot.TalonSet;
import frc.robot.subsystems.DriveSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command implements PIDSource, PIDOutput {

    PIDController pid;
    PIDSourceType pidSourceType;
    PIDSource source;
    AHRS gyro;
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;
    double setPoint;
    StringBuilder sb;

    public TurnToAngle(double kp, double ki, double kd, double kf,double angle) {
        pid = new PIDController(kp, ki, kd, kf, this, this);
        pidSourceType = PIDSourceType.kDisplacement;
        pid.setSetpoint(angle);
        pid.setPercentTolerance(5);
        requires(driveSubsystem);
    }

    @Override
    protected void initialize() {
        System.out.println("TurnToAngle init");
        gyro = driveSubsystem.getGyro();
        gyro.reset();
        setSetpoint(setPoint);
        setOutputRange(-1, 1);
        enable();        
    }

    
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        sb.append(pid.onTarget());
        sb.append(" ");
        return pid.onTarget();
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        System.out.println("TurnToangle end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        System.out.println("TurnToAngle interrupted");
    }












    public PIDController getPID(){
        return this.pid;
    }
    public TurnToAngle setP(double kp) {
        pid.setP(kp);
        return this;
    }

    public TurnToAngle setI(double ki) {
        pid.setI(ki);
        return this;
    }

    public TurnToAngle setD(double kd) {
        pid.setD(kd);
        return this;
    }

    /* public EncoderMotorPID setDistancePerTick(double dpt) {
        encoder.setDistancePerPulse(dpt);
        return this;
    } */

    public TurnToAngle setOutputRange(double l, double h) {
        pid.setOutputRange(l,h);
        return this;
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public void pidWrite(double speed) {
        driveSubsystem.tankDrive(speed, -speed);
    }

    public void setPIDSourceType(PIDSourceType t) {
        pidSourceType = t;
    }

    public PIDSourceType getPIDSourceType() {
        return gyro.getPIDSourceType();
    }

    public double pidGet() {
        return gyro.pidGet();
    }

    public TurnToAngle setSetpoint(double sp) {
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
