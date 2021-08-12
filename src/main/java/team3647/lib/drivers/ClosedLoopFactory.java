/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Config pid loops and feed forward for motors.
 */
public class ClosedLoopFactory {
    private final static int kTimeoutms = 100;

    public static class ClosedLoopConfig {
        public double kEncoderAccelerationToUnits;
        public double kEncoderVelocityToRPM;
        public double kEncoderTicksToUnits;
        public double kP, kI, kD;
        public double kS, kV, kA;
        public double maxVelocity;
        public double maxAcceleration;
        public double positionThreshold;
        public double velocityThreshold;
        public boolean sensorInverted;

        public ClosedLoopConfig() {
            kEncoderAccelerationToUnits = 1;
            kEncoderVelocityToRPM = 1;
            kEncoderTicksToUnits = 1;
            kP = 0;
            kI = 0;
            kD = 0;
            kS = 0;
            kV = 0;
            kA = 0;
            maxVelocity = 0;
            maxAcceleration = 0;
            positionThreshold = 0;
            velocityThreshold = 0;
            sensorInverted = false;
        }

        public ClosedLoopConfig(double kEncoderAccelerationToUnits, double kEncoderVelocityToRPM,
                double kEncoderTicksToUnits, double kP, double kI, double kD, double kS, double kV,
                double kA, double[] feedForwardArr, double maxVelocity, double maxAcceleration,
                double positionThreshold, double velocityThreshold, boolean sensorInverted) {
            this.kEncoderAccelerationToUnits = kEncoderAccelerationToUnits;
            this.kEncoderVelocityToRPM = kEncoderVelocityToRPM;
            this.kEncoderTicksToUnits = kEncoderTicksToUnits;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.positionThreshold = positionThreshold;
            this.velocityThreshold = velocityThreshold;
            this.sensorInverted = sensorInverted;
        }

        public ClosedLoopConfig positionThreshold(double positionThreshld) {
            this.positionThreshold = positionThreshld;
            return this;
        }

        public ClosedLoopConfig velocityThrehsold(double velocityThreshold) {
            this.velocityThreshold = velocityThreshold;
            return this;
        }

        public ClosedLoopConfig maxAcceleration(double maxAcceleration) {
            this.maxAcceleration = maxAcceleration;
            return this;
        }

        public ClosedLoopConfig maxVelocity(double maxVelocity) {
            this.maxVelocity = maxVelocity;
            return this;
        }

        public ClosedLoopConfig configPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public ClosedLoopConfig configFeedForward(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        public ClosedLoopConfig encoderAccelerationToUnits(double encoderAccelerationToUnits) {
            this.kEncoderAccelerationToUnits = encoderAccelerationToUnits;
            return this;
        }

        public ClosedLoopConfig encoderVelocityToRPM(double encoderVelocityToRPM) {
            this.kEncoderVelocityToRPM = encoderVelocityToRPM;
            return this;
        }

        public ClosedLoopConfig encoderTicksToUnits(double encoderTicksToUnits) {
            this.kEncoderTicksToUnits = encoderTicksToUnits;
            return this;
        }

        public ClosedLoopConfig sensorInverted(boolean sensorInverted) {
            this.sensorInverted = sensorInverted;
            return this;
        }
    }

    public static ClosedLoopConfig DEFAULT = new ClosedLoopConfig();

    private static void handleCANError(int id, CANError error, String message) {
        if (error != CANError.kOk) {
            DriverStation.reportError("Could not configure spark id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    public static CANPIDController createSparkMaxPIDController(CANSparkMax master,
            CANEncoder feedbackDevice, ClosedLoopConfig config, int slot) {
        CANPIDController controller = master.getPIDController();
        configSparkMaxPIDController(controller, master, feedbackDevice, config, slot);
        return controller;
    }

    public static void configSparkMaxPIDController(CANPIDController controller, CANSparkMax master,
            CANEncoder feedbackDevice, ClosedLoopConfig config, int slot) {
        int id = master.getDeviceId();
        double maxVelocityTicks = config.maxVelocity / config.kEncoderVelocityToRPM;
        double maxAccelerationTicks = config.maxVelocity / config.kEncoderAccelerationToUnits;

        handleCANError(id, controller.setP(config.kP, slot), "set P");
        handleCANError(id, controller.setI(config.kI, slot), "set I");
        handleCANError(id, controller.setD(config.kD, slot), "set D");
        handleCANError(id, controller.setSmartMotionMaxAccel(maxAccelerationTicks, slot),
                "set smart motion accel");
        handleCANError(id, controller.setSmartMotionMaxVelocity(maxVelocityTicks, slot),
                "set smart motion max vel");
        handleCANError(id, controller.setFeedbackDevice(feedbackDevice), "set feedback device");
    }

    private static void handleCANError(int id, ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError("Could not configure talon id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    public static void configTalonPIDController(TalonSRX talon, FeedbackDevice feedbackDevice,
            ClosedLoopConfig config, int slot) {
        int id = talon.getDeviceID();
        handleCANError(id, talon.configSelectedFeedbackSensor(feedbackDevice, 0, kTimeoutms),
                "config feedback device");

        int maxVelocityTicks = (int) (config.maxVelocity / config.kEncoderVelocityToRPM);
        int maxAccelerationTicks = (int) (config.maxVelocity / config.kEncoderAccelerationToUnits);
        handleCANError(id, talon.config_kP(slot, config.kP, kTimeoutms), "set kP");
        handleCANError(id, talon.config_kI(slot, config.kI, kTimeoutms), "set kI");
        handleCANError(id, talon.config_kD(slot, config.kD, kTimeoutms), "set kD");
        handleCANError(id, talon.configMotionAcceleration(maxAccelerationTicks, kTimeoutms),
                "set acceleration");
        handleCANError(id, talon.configMotionCruiseVelocity(maxVelocityTicks, kTimeoutms),
                "set velocity");
        talon.setSensorPhase(config.sensorInverted);
    }
}
