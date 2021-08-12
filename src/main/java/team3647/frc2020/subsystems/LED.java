/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

/**
 * Add your docs here. c - green b - red a - blue
 */
public class LED implements PeriodicSubsystem {

    // private final CANifier m_CANifier;

    public class PeriodicIO {
        public double redDemand = 0;
        public double greenDemand = 0;
        public double blueDemand = 0;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    public LED(int canifierPin) {
        // m_CANifier = new CANifier(canifierPin);
    }

    public void setRed(double output) {
        periodicIO.redDemand = output;
    }

    public void setBlue(double output) {
        periodicIO.blueDemand = output;
    }

    public void setGreen(double output) {
        periodicIO.greenDemand = output;
    }

    public void set(double redOutput, double greenOutput, double blueOutput) {
        setRed(redOutput);
        setGreen(greenOutput);
        setBlue(blueOutput);
    }

    public double getGreen() {
        return periodicIO.greenDemand;
    }

    public double getRed() {
        return periodicIO.redDemand;
    }

    public double getBlue() {
        return periodicIO.blueDemand;
    }

    @Override
    public void writePeriodicOutputs() {
        // m_CANifier.setLEDOutput(periodicIO.redDemand, LEDChannel.LEDChannelB);
        // m_CANifier.setLEDOutput(periodicIO.greenDemand, LEDChannel.LEDChannelC);
        // m_CANifier.setLEDOutput(periodicIO.blueDemand, LEDChannel.LEDChannelA);
    }

    @Override
    public String getName() {
        return "LED";
    }
}
