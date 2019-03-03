package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.pids.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;


import com.kauailabs.navx.frc.AHRS;


/**
 * Add your docs here.
 */
public class GoToLine extends CommandGroup {
    
    public GoToLine(){
        addSequential(new TurnToAngle());
        addSequential(new GoToDistance());
    }
    
}