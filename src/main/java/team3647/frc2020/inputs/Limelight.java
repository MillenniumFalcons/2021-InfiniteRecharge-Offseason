/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.inputs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    // NetworkTable is the class used to grab values from the Limelight Network
    // Table
    private final NetworkTable table;

    private final String ip;

    public enum Data {
        VALID_TARGET("tv"), X("tx"), Y("ty"), AREA("ta"), SKEW("ts"), LATNECY("tl");

        public final String str;

        Data(String str) {
            this.str = str;
        }
    }

    public enum LEDMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        public final int asInt;

        LEDMode(int mode) {
            this.asInt = mode;
        }
    }

    public enum CamMode {
        VISION(0), DRIVER(1);

        public final int asInt;

        CamMode(int mode) {
            this.asInt = mode;
        }
    }

    public enum StreamMode {
        SideBySide(0), Limelight(1), USBCam(2);

        public final int asInt;

        StreamMode(int mode) {
            this.asInt = mode;
        }
    }

    // used to initalize the main, important things
    public Limelight(String ip) {
        // initializing the network table to grab values from limelight
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.ip = ip;
    }

    public String toString() {
        return ip;
    }

    public void setPipeline(int pipeline) {
        set("pipeline", pipeline);
    }


    public void ledMode(LEDMode mode) {
        set("ledMode", mode.asInt);
    }

    public void camMode(CamMode mode) {
        set("camMode", mode.asInt);
    }

    public void streamMode(StreamMode stream) {
        set("stream", stream.asInt);
    }

    public double get(Data input) {
        return table.getEntry(input.str).getDouble(0);
    }

    private void set(String input, int input2) {
        table.getEntry(input).setNumber(input2);
    }

}
