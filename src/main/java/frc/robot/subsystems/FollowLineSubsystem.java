/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ConstantsMap;

/**
 * Add your docs here.
 */
public class FollowLineSubsystem extends Subsystem{
    I2C i2c;
    public FollowLineSubsystem(){
        System.out.println("Line Follow Subsystem Init");
        i2c = new I2C(Port.kOnboard, 0x09);
        

    }

    @Override
    public void initDefaultCommand() {
        
    }
    public String getData(){
        byte[] fromSensor = new byte[16];
        i2c.readOnly(fromSensor,16);
        int[] data = new int[8];
        for(int i= 0;i<8;i++){
            data[i] = fromSensor[2*i] & 0xFF;
        }
        return Arrays.toString(data);
    }

    public int[] getRawData(){
        byte[] fromSensor = new byte[16];
        i2c.readOnly(fromSensor,16);
        int[] data = new int[8];
        for(int i= 0;i<8;i++){
            data[i] = fromSensor[2*i] & 0xFF;
        }
        return data;
    }

    //gets data from camera srip, strip 1 is front camera, strip 2 is back camera
    public boolean[] getLineData(int strip) {
        boolean[] sensors = new boolean[8];

        byte[] fromSensor = new byte[16];
        i2c.readOnly(fromSensor,16);
        int[] data = new int[8];

        for(int i= 0;i<8;i++){
            data[i] = fromSensor[2*i] & 0xFF;
        }

        for(int i = 0; i < 8; i++){
            sensors[i] = data[i] > ConstantsMap.BLACK_WHITE_CUTOFF;
        }      

        return sensors;
    }
    public double getLineAverage(int strip){
        boolean[] isFrontCameraOnStrip = getLineData(strip);
        
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
