                package frc.robot.pids;

import frc.robot.ConstantsMap;
import frc.robot.Gains;
import frc.robot.Robot;
import frc.robot.TalonSet;
import frc.robot.maths.Vector2;
import frc.robot.subsystems.DriveSubsystem;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngle extends Command implements PIDSource, PIDOutput {

    PIDController pid;
    PIDSourceType pidSourceType;
    PIDSource source;
    AHRS gyro;
    DriveSubsystem driveSubsystem = Robot.driveSubsystem;
    double setPoint;

    public TurnToAngle() {
        requires(driveSubsystem);
        Gains g = ConstantsMap.turnGains;
        pid = new PIDController(g.kP, g.kI, g.kD, g.kF, this, this);
        pidSourceType = PIDSourceType.kDisplacement;
        pid.setPercentTolerance(.5);
        pid.setInputRange(-180, 180);
        pid.setOutputRange(-1, 1);

    }        

    @Override
    protected void initialize() {
        System.out.println("TurnToAngle init");
        gyro = driveSubsystem.getGyro();
        double startAngle = SmartDashboard.getNumber("LineStartAngle", 0);
        double startDistance = SmartDashboard.getNumber("LineStartDistance", 0);
        double backAngle = SmartDashboard.getNumber("LineBackAngle", 0);
        double backDistance = SmartDashboard.getNumber("LineStartDistance", 0);
        Vector2 start = Vector2.fromPolar(startAngle, startDistance);
        Vector2 back = Vector2.fromPolar(backAngle, backDistance);
        Vector2 line = start.add(back.multiply(-1));   
        
        Vector2 extendedLine = back.proj(line);
        Vector2 target = start.add(extendedLine.multiply(-1));   
        setPoint = Math.atan2(target.x, target.y) * 180/Math.PI;

        System.out.println("Angle: " + setPoint);
        gyro.reset();
        pid.setSetpoint(setPoint);
        pid.enable();        
    }

    
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //System.out.print(pid.getSetpoint() + " " + gyro.getAngle());
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return pid.onTarget();
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        pid.disable();

        System.out.println("TurnToangle end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        pid.disable();
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
