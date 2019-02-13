/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.HatchGrabberSubsystem;

public class HatchGrabberCommand extends Command {
    HatchGrabberSubsystem hatchGrabberSubsystem = Robot.hatchGrabberSubsystem;
    private boolean grabberOut = false;
    private boolean hatchReleased = false;
    private double lastPressedG = 0;
    private double lastPressedH = 0;

    public HatchGrabberCommand() {
        requires(hatchGrabberSubsystem);
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // if (XboxMap.ejectHatch()) {
        //     hatchGrabberSubsystem.releaseHatch();
        // } else if (XboxMap.retractEjectors()) {
        //     hatchGrabberSubsystem.retractEjectors();
        // } else if (XboxMap.extendGrabber()) {
        //     hatchGrabberSubsystem.extendGrabber();
        // } else if (XboxMap.retractGrabber()) {
        //     hatchGrabberSubsystem.retractGrabber();
        // }

        if(XboxMap.grabberPiston() && (System.currentTimeMillis() - lastPressedG) > 500) {
            lastPressedG = System.currentTimeMillis();
            if(grabberOut) {
                hatchGrabberSubsystem.extendGrabber();
                grabberOut = !grabberOut;
            }
            else {
                hatchGrabberSubsystem.retractGrabber();
                grabberOut = !grabberOut;
            }
        }

        if(XboxMap.hatchPiston() && (System.currentTimeMillis() - lastPressedH) > 500) {
            lastPressedH = System.currentTimeMillis();
            if(hatchReleased) {
                hatchGrabberSubsystem.releaseHatch();
            }
            else {
                hatchGrabberSubsystem.retractEjectors();
            }
        }
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
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
