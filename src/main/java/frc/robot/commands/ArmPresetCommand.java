package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ConstantsMap;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.ArmSubsystem2;

public class ArmPresetCommand extends Command {
    ArmSubsystem2 armSubsystem = Robot.armSubsystem2;

    public ArmPresetCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(armSubsystem);
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (XboxMap.cargoPreset1()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.CARGO_PRESET_1_ANGLE);
        } else if (XboxMap.cargoPreset2()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.CARGO_PRESET_2_ANGLE);
        } else if (XboxMap.cargoPreset3()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.CARGO_PRESET_3_ANGLE);
        } else if (XboxMap.hatchPreset1()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.HATCH_PRESET_1_ANGLE);
        } else if (XboxMap.hatchPreset2()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.HATCH_PRESET_2_ANGLE);
        } else if (XboxMap.hatchPreset3()) {
            armSubsystem.setShoulderSetPoint(ConstantsMap.HATCH_PRESET_3_ANGLE);
        }
        armSubsystem.setWristSetPoint(-armSubsystem.getShoulderEncoderAngle());
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // TODO: will this work or should we use tolerance equality?
        //return armSubsystem.getShoulderEncoderAngle() == gotoAngle;
        return false;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
