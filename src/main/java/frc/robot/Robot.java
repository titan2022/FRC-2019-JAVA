/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmCommand2;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ArmZero;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HatchGrabberCommand;
import frc.robot.commands.WristZero;
import frc.robot.subsystems.ArmSubsystem2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HatchGrabberSubsystem;

/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the TimedRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public static OI oi;
    
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static ArmSubsystem2 armSubsystem2 = new ArmSubsystem2();
    
    public static HatchGrabberSubsystem hatchGrabberSubsystem = new HatchGrabberSubsystem();

    UsbCamera camera;
    Command autonomousCommand;
    Command driveCommand;
    Command followLineCommand;
    Command armCommand;
    Command hgCommand;
    Command zeroWrist;

    SendableChooser<Command> chooser = new SendableChooser<>();
    CameraServer server;
    Compressor compressor;
    public boolean debugMode;
    Command preset;
    /**
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
    */
    @Override
    public void robotInit() {
        oi = new OI();
        
        // chooser.addOption("My Auto", new MyAutoCommand());
        driveCommand = new DriveCommand();
        //followLineCommand = new FollowLineCommand();
        hgCommand = new HatchGrabberCommand();
        armCommand = new ArmCommand2();
        
        server = CameraServer.getInstance();
        debugMode = false;
        
        //server.startAutomaticCapture("Ground",0);
    }
    public boolean getDebug(){
        return debugMode;
    }
    
    /**
    * This function is called every robot packet, no matter the mode. Use
    * this for items like diagnostics that you want ran during disabled,
    * autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    
    public void robotPeriodic() {
        //Print out encoder values for testing on Arm leveling
        SmartDashboard.putNumber("Shoulder Angle", armSubsystem2.getShoulderEncoderAngle());
        SmartDashboard.putNumber("Shoulder Set", armSubsystem2.getShoulderSetPoint());
        SmartDashboard.putNumber("Wrist Set", armSubsystem2.getWristSetPoint());

        SmartDashboard.putNumber("Wrist Angle", armSubsystem2.getWristEncoderAngle());
        SmartDashboard.putBoolean("Wrist Limit", armSubsystem2.getWristLowerLimit());
        SmartDashboard.putBoolean("Arm Limit", armSubsystem2.getShoulderLowerLimit());

        SmartDashboard.putBoolean("Debug Mode", debugMode);

        SmartDashboard.putBoolean("Manual Mode", armSubsystem2.getManualMode());
        SmartDashboard.putBoolean("Level Mode", armSubsystem2.getLevelMode());



        armSubsystem2.checkShoulderLimits();
        armSubsystem2.checkWristLimits();

        if(ControlPanelMap.toggleDebug()){
            debugMode = !debugMode;
            oi.unbindButtons();
            if(debugMode){
                oi.debugMode();
            }
            else{
                oi.normalMode();
            }
            
        }
        if(ControlPanelMap.toggleManual() && debugMode){
            armSubsystem2.toggleManualMode();
        }
        if(ControlPanelMap.toggleLevel() && debugMode){
            armSubsystem2.toggleLevelMode();
        }

    }
    
    /**
    * This function is called once each time the robot enters Disabled mode.
    * You can use it to reset any subsystem information you want to clear when
    * the robot is disabled.
    */
    @Override
    public void disabledInit() {
        XboxMap.stopRumble();

        armSubsystem2.setShoulderJointSpeed(0);
    }
    
    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }
    
    /**
    * This autonomous (along with the chooser code above) shows how to select
    * between different autonomous modes using the dashboard. The sendable
    * chooser code works with the Java SmartDashboard. If you prefer the
    * LabVIEW Dashboard, remove all of the chooser code and uncomment the
    * getString code to get the auto name from the text box below the Gyro
    *
    * <p>You can add additional auto modes by adding additional commands to the
    * chooser code above (like the commented example) or additional comparisons
    * to the switch structure below with additional strings & commands.
    */
    
    @Override
    public void autonomousInit() {
        
        
        /*
        * String autoSelected = SmartDashboard.getString("Auto Selector",
        * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
        * = new MyAutoCommand(); break; case "Default Auto": default:
        * autonomousCommand = new ExampleCommand(); break; }
        */
        
        // schedule the autonomous command (example)
        armSubsystem2.setShoulderSetPoint(armSubsystem2.getShoulderEncoderAngle());

        new WristZero().start();;

    }
    
    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.


        hgCommand.start();
        armSubsystem2.setShoulderSetPoint(armSubsystem2.getShoulderEncoderAngle());
        //armCommand.start();
        
        //driveCommand.start();
        
    }
    
    
    /**
    * This function is called periodically during operator control.
    */
    @Override
    public void teleopPeriodic() {
        if(debugMode){
            if(ControlPanelMap.inTake()){
                new ArmZero();
            }
            if(ControlPanelMap.outTake()){
                new WristZero();
            }
        }
        /* if(XboxMap.runFollowLineCommand()){
            if(followLineCommand.isRunning()){
                followLineCommand.cancel();
                driveCommand.start();
            }
            else{
                driveCommand.cancel();
                followLineCommand.start();
            }
            
        }    */
        
        
        /* if(XboxMap.toggleArmControl()){
            if(armCommand.isRunning()){
                armCommand.cancel();
                preset.start();
            }
            else{
                preset.cancel();
                armCommand.start();
                
            }
        } 
        if(XboxMap.zeroWrist()){
            if(armCommand.isRunning()){
                armCommand.cancel();
                zeroWrist.start();
            }
            else{
                zeroWrist.cancel();
                armCommand.start();
                
            }
        }  */
        SmartDashboard.putBoolean("Arm Control", armCommand.isRunning());
        Scheduler.getInstance().run();
    }
    
    
    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }
}
