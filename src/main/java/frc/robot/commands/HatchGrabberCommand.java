/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsMap;
import frc.robot.ControlPanelMap;
import frc.robot.Robot;
import frc.robot.XboxMap;
import frc.robot.subsystems.HatchGrabberSubsystem;

public class HatchGrabberCommand extends Command {
    HatchGrabberSubsystem hatchGrabberSubsystem = Robot.hatchGrabberSubsystem;
    private boolean grabberOut = false;
    private boolean hatchReleased = false;
    private boolean hatchMode;
    private double lastPressedG = 0;
    private double lastPressedH = 0;

    public HatchGrabberCommand() {
        requires(hatchGrabberSubsystem);
    }
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        hatchMode = false;
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        if(ControlPanelMap.setBallMode()){
            hatchMode = false;
        }
        if(ControlPanelMap.setHatcheMode()){
            hatchMode = true;
            hatchGrabberSubsystem.stopWheels();

        }
        if(hatchMode){
            hatchGrabberSubsystem.extendHatch();
            if(ControlPanelMap.outTake()) {            
                hatchGrabberSubsystem.ejectHatch();
                hatchMode = false;
                hatchGrabberSubsystem.retractHatch();
                

            }
            else{
                hatchGrabberSubsystem.unEjectHatch();
            }
        }
        else{
            hatchGrabberSubsystem.retractHatch();
            
            if(ControlPanelMap.outTake()) {            
                hatchGrabberSubsystem.outakeWheels();
            }
            else if(ControlPanelMap.inTake()){
                hatchGrabberSubsystem.unEjectHatch();

                hatchGrabberSubsystem.intakeWheels();
            }
            else{
                hatchGrabberSubsystem.stopWheels();
            }
        }
        SmartDashboard.putBoolean("Hatch Mode",hatchMode);
        

        
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
