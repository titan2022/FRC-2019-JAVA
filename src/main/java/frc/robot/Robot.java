/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmCommand2;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GoHome;
import frc.robot.commands.HatchGrabberCommand;
import frc.robot.commands.WristForceZero;
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

    UsbCamera groundCam,grabCam;
   // HttpCamera grabCam;
    Command autonomousCommand;
    Command driveCommand;
    Command followLineCommand;
    Command armCommand;
    Command hgCommand;
    boolean groundCamSelected;
    SendableChooser<Command> chooser = new SendableChooser<>();
    CameraServer server;
    Compressor compressor;
    public boolean debugMode;
    Command preset;
    int count = 0;
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
        
        groundCam = new UsbCamera("GroundCam", 0);
        grabCam = new UsbCamera("GrabCam", 1);
        grabCam.setResolution(600, 480);
        
        server.startAutomaticCapture(groundCam);
        groundCamSelected = true;
        //grabCam = new HttpCamera("Vision Process", "http://10.20.22.207:1181/?action=stream/mjpeg");

    }
    public void switchCamera(){
        System.out.println("Switched Cam");
        if(!groundCamSelected){
            server.getServer().setSource(groundCam);
            groundCamSelected = true;
        }
        else{
            server.getServer().setSource(grabCam);
            groundCamSelected = false;  
        }
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
        SmartDashboard.putNumber("Shoulder Right Angle", armSubsystem2.getRightShoulderEncoderAngle());
        SmartDashboard.putNumber("Shoulder Left Angle", armSubsystem2.getLeftShoulderEncoderAngle());

        SmartDashboard.putNumber("Shoulder Right Set", armSubsystem2.getRightShoulderSetPoint());
        SmartDashboard.putNumber("Shoulder Left Set", armSubsystem2.getRightShoulderSetPoint());

        SmartDashboard.putNumber("Wrist Set", armSubsystem2.getWristSetPoint());

        SmartDashboard.putNumber("Wrist Angle", armSubsystem2.getWristEncoderAngle());
        SmartDashboard.putBoolean("Wrist Limit", armSubsystem2.getWristLowerLimit());
        SmartDashboard.putBoolean("Arm Limit", armSubsystem2.getShoulderLowerLimit());
        SmartDashboard.putBoolean("Debug Mode", debugMode);

        SmartDashboard.putBoolean("Manual Mode", armSubsystem2.getManualMode());
        SmartDashboard.putBoolean("Level Mode", armSubsystem2.getLevelMode());

        SmartDashboard.putBoolean("Tipping", driveSubsystem.checkTip());


        SmartDashboard.putBoolean("Left At Set", armSubsystem2.isLeftShoulderAtSetPoint());
        SmartDashboard.putBoolean("Right At Set", armSubsystem2.isRightShoulderAtSetPoint());
        SmartDashboard.putBoolean("Shoulder At Set", armSubsystem2.isShoulderAtSetPoint());


        //SmartDashboard.putBoolean("Right Shoulder Set", armSubsystem2.getLevelMode());


        armSubsystem2.checkShoulderLimits();
        armSubsystem2.checkWristLimits();

        count++;
        if(count>10){
            SmartDashboard.putBoolean("Drifting", driveSubsystem.checkDrift());
        
            SmartDashboard.putBoolean("On Fire", driveSubsystem.isOnFire());
            SmartDashboard.putBoolean("Cruising Altitude", driveSubsystem.isCruisingAltitude());
            SmartDashboard.putBoolean("Is Probably About to Rain", driveSubsystem.isProbablyAboutToRain());

            SmartDashboard.putBoolean("Drive Stall Detected", driveSubsystem.isStalled());
            SmartDashboard.putBoolean("Shoulder Stall Detected", armSubsystem2.isShoulderStalled());
            SmartDashboard.putBoolean("Wrist Stall Detected", armSubsystem2.isWristStalled());

            

            count = 0;

        }
        
        


        
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
        if(ControlPanelMap.forceZero() && debugMode){
            armSubsystem2.zeroShoulder();
            armSubsystem2.zeroWrist();
        }
        if((ControlPanelMap.switchCam()  ||  XboxMap.switchCam())){
            switchCamera();
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

        new WristForceZero().start();
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


        armSubsystem2.setShoulderSetPoint(armSubsystem2.getShoulderEncoderAngle());
        new WristForceZero().start();

        
        //armCommand.start();
        
        //driveCommand.start();
        
    }
    
    
    /**
    * This function is called periodically during operator control.
    */
    @Override
    public void teleopPeriodic() {
        
        SmartDashboard.putBoolean("Arm Control", armCommand.isRunning());

        if(driveSubsystem.checkTip()){
            new GoHome().start();
        }
        
        Scheduler.getInstance().run();
    }
    
    
    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }
}
