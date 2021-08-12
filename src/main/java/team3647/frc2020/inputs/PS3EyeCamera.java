/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.inputs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Camera Class to get "working distance" or "projected distance" using the following constants and
 * variables
 */
public class PS3EyeCamera {
    private NetworkTable table;
    private NetworkTableEntry yaw;
    private NetworkTableEntry pitch;
    private NetworkTableEntry isValid;


    public PS3EyeCamera(String camIP, String camName) {
        table = NetworkTableInstance.getDefault().getTable(camIP).getSubTable(camName);
        yaw = table.getEntry("yaw");
        pitch = table.getEntry("pitch");
        isValid = table.getEntry("isValid");
    }

    public double getYaw() {
        return yaw.getDouble(0.0);
    }

    public double getPitch() {
        return pitch.getDouble(0.0);
    }

    public boolean isValid() {
        return isValid.getBoolean(false);
    }
}
