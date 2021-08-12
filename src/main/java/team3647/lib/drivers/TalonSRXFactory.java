package team3647.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application. (254)
 */
public class TalonSRXFactory {

    private final static int kTimeoutms = 100;

    public static class Configuration {
        public final int CANID;
        public final boolean inverted;

        public boolean enableCurrentLimiting = false;
        public int peakCurrent = 0;
        public int peakCurrentDuration = 0;
        public int continuousCurrent = 0;
        public NeutralMode neutralMode = NeutralMode.Coast;
        public boolean voltageCompensation = false;
        public double nominalVoltage = 0.0;
        public double maxOutput = 1;
        public double minOutput = -1;

        public double secondsFromNeutralToFull = 0;

        public Configuration(int CANID, boolean inverted) {
            this.CANID = CANID;
            this.inverted = inverted;
        }

        public Configuration currentLimiting(boolean enable, int peakCurrent,
                int peakCurrentDuration, int continuousCurrent) {
            enableCurrentLimiting = enable;
            this.peakCurrent = peakCurrent;
            this.peakCurrentDuration = peakCurrentDuration;
            this.continuousCurrent = continuousCurrent;
            return this;
        }

        public Configuration voltageCompensation(boolean enable, double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            this.voltageCompensation = enable;
            return this;
        }

        public Configuration neutralMode(NeutralMode mode) {
            this.neutralMode = mode;
            return this;
        }

        /**
         * @param maxOuput is [0, 1]
         */
        public Configuration configMaxOutput(double maxOutput) {
            if (maxOutput < 1 && maxOutput > 0) {
                this.maxOutput = maxOutput;
            }
            return this;
        }

        /**
         * @param minOutpu is [-1, 0]
         */
        public Configuration configMaxReverseOutput(double minOutput) {
            if (minOutput < 0 && minOutput > -1) {
                this.minOutput = minOutput;
            }
            return this;
        }

        public Configuration configOpenLoopRampRate(double secondsFromNeutralToFull) {
            this.secondsFromNeutralToFull = secondsFromNeutralToFull;
            return this;
        }

        public static Configuration mirrorWithCANID(Configuration config, int CANID) {

            return new Configuration(CANID, config.inverted)
                    .currentLimiting(config.enableCurrentLimiting, config.peakCurrent,
                            config.peakCurrentDuration, config.continuousCurrent)
                    .voltageCompensation(config.voltageCompensation, config.nominalVoltage)
                    .neutralMode(config.neutralMode).configMaxOutput(config.maxOutput)
                    .configMaxReverseOutput(config.minOutput);
        }

    }

    private static final Configuration DEFAULT = new Configuration(0, false);

    private static void handleCANError(int id, ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError("Could not configure talon id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonSRX createDefaultTalon() {
        return createTalon(DEFAULT);
    }

    public static TalonSRX createTalon(Configuration config) {
        TalonSRX talon = new TalonSRX(config.CANID);
        talon.set(ControlMode.PercentOutput, 0.0);
        talon.setInverted(config.inverted);
        handleCANError(config.CANID, talon.configFactoryDefault(kTimeoutms),
                "restore factory defaults");
        talon.clearStickyFaults(kTimeoutms);

        talon.enableCurrentLimit(config.enableCurrentLimiting);
        handleCANError(config.CANID, talon.configPeakCurrentLimit(config.peakCurrent, kTimeoutms),
                "set peak current");
        handleCANError(config.CANID,
                talon.configPeakCurrentDuration(config.peakCurrentDuration, kTimeoutms),
                "set peak current duration");
        handleCANError(config.CANID,
                talon.configContinuousCurrentLimit(config.continuousCurrent, kTimeoutms),
                "set continuous current");

        talon.enableVoltageCompensation(config.voltageCompensation);
        handleCANError(config.CANID,
                talon.configVoltageCompSaturation(config.nominalVoltage, kTimeoutms),
                "set nominal voltage");

        handleCANError(config.CANID, talon.configPeakOutputForward(config.maxOutput, kTimeoutms),
                "set max forward output");

        handleCANError(config.CANID, talon.configPeakOutputReverse(config.minOutput, kTimeoutms),
                "set max reverse output");

        handleCANError(config.CANID,
                talon.configOpenloopRamp(config.secondsFromNeutralToFull, kTimeoutms),
                "config open loop ramp rate");
        return talon;
    }
}
