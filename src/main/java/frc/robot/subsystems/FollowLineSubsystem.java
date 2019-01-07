/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class FollowLineSubsystem extends Subsystem{

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    //gets data from camera srip, strip 1 is front camera, strip 2 is back camera
    public boolean[] getCameraData(int strip) {
        boolean[] sensors = new boolean[8];

        //get shit like bossP        

        return sensors;
    }

    public double getCameraAverage(int strip){
        boolean[] isFrontCameraOnStrip = getCameraData(strip);
        
        double sum = 0.0d;
        int camerasOn = 0;

        for(int i = 0; i < 8; i++){
            if(isFrontCameraOnStrip[i]){
                sum+=i;
                camerasOn++;
            }
        }
        
        return sum/camerasOn;
    }

}
