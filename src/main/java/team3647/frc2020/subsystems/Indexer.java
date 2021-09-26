/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.IndexerSignal;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;
import team3647.lib.wpi.HALMethods;

/**
 * feeder boi.
 */
public class Indexer implements PeriodicSubsystem {

    private final VictorSPX leftVertical;
    private final VictorSPX tunnel;
    private final VictorSPX horizontalRollers;
    private final TalonSRX rightVertical;

    private final int leftVerticalPDPSlot;
    private final int tunnelPDPSlot;
    private final int rollersPDPSlot;

    private final DigitalInput bannerSensor;
    private boolean bannerSensorValue;

    private final Function<Integer, Double> getCurrent;

    public Indexer(VictorSPXFactory.Configuration leftVerticalRollersConfig,
            TalonSRXFactory.Configuration rightVerticalRollersConfig, VictorSPXFactory.Configuration tunnelConfig,
            VictorSPXFactory.Configuration rollersConfig, int bannerSensorPin, Function<Integer, Double> getCurrent) {
        boolean error = false;
        if (leftVerticalRollersConfig == null) {
            HALMethods.sendDSError("funnel config was null");
            error = true;
        }
        if (tunnelConfig == null) {
            HALMethods.sendDSError("tunnel config was null");
            error = true;
        }
        if (rollersConfig == null) {
            HALMethods.sendDSError("rollers config was null");
            error = true;
        }
        if (getCurrent == null) {
            HALMethods.sendDSError(getName() + "getcurrent was null");
            error = true;
        }

        if (error) {
            throw new NullPointerException("1 or more of the arguments to Indexer constructor were null");
        } else {
            leftVertical = VictorSPXFactory.createVictor(leftVerticalRollersConfig);
            tunnel = VictorSPXFactory.createVictor(tunnelConfig);
            horizontalRollers = VictorSPXFactory.createVictor(rollersConfig);
            rightVertical = TalonSRXFactory.createTalon(rightVerticalRollersConfig);
            this.getCurrent = getCurrent;
        }

        leftVerticalPDPSlot = leftVerticalRollersConfig.pdpSlot;
        tunnelPDPSlot = tunnelConfig.pdpSlot;
        rollersPDPSlot = rollersConfig.pdpSlot;

        bannerSensor = new DigitalInput(bannerSensorPin);
    }

    /**
     * @return the rollersPDPSlot
     */
    public int getRollersPDPSlot() {
        return rollersPDPSlot;
    }

    /**
     * @return the tunnelPDPSlot
     */
    public int getTunnelPDPSlot() {
        return tunnelPDPSlot;
    }

    /**
     * @return the funnelPDPSlot
     */
    public int getLeftVerticalPDPSlot() {
        return leftVerticalPDPSlot;
    }

    public void set(IndexerSignal signal) {
        if (signal == null) {
            HALMethods.sendDSError("Indexer signal was null!");
            return;
        }
        if (getCurrent.apply(leftVerticalPDPSlot) > 30) {
            leftVertical.set(ControlMode.PercentOutput, signal.getLeftVerticalOutput() * .5);
        } else {
            leftVertical.set(ControlMode.PercentOutput, signal.getLeftVerticalOutput());
        }

        if (Math.abs(rightVertical.getSupplyCurrent()) > 30) {
            rightVertical.set(ControlMode.PercentOutput, signal.getRightVerticalOutput() * .5);
        } else {
            rightVertical.set(ControlMode.PercentOutput, signal.getRightVerticalOutput());
        }

        if (getCurrent.apply(tunnelPDPSlot) > 30) {
            tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput() * .5);
        } else {
            tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput());
        }
        horizontalRollers.set(ControlMode.PercentOutput, signal.getHorizontalRollersOutput());
    }

    @Override
    public void periodic() {
        bannerSensorValue = !bannerSensor.get();
    }

    public boolean getBannerSensorValue() {
        return bannerSensorValue;
    }

    @Override
    public void end() {
        set(IndexerSignal.STOP);
    }

    @Override
    public String getName() {
        return "Indexer";
    }
}