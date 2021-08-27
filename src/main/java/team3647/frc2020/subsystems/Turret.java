/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpiutil.math.MathUtil;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;

/**
 * rotate.
 */
public class Turret extends TalonSRXSubsystem {
    private final double kMaxRotationDeg;
    private final double kMinRotationDeg;
    private final DigitalInput limitSwitch;
    private boolean isOnLimitSwitch = false;
    private boolean isAiming = false;

    public Turret(TalonSRXFactory.Configuration masterConfig, ClosedLoopConfig pidConfig,
            double maxRotationDeg, double minRotationDeg, int limitSwitchPin) {
        super(masterConfig, pidConfig);
        enablePositionFiltering();
        kMaxRotationDeg = maxRotationDeg;
        kMinRotationDeg = minRotationDeg;
        limitSwitch = new DigitalInput(limitSwitchPin);
        setEncoderPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        isOnLimitSwitch = !limitSwitch.get();
        if (isOnLimitSwitch) {
            // end();
        }
    }

    @Override
    protected void resetEncoder() {
        setEncoderPositionUnits(180);
    }

    /**
     * https://stackoverflow.com/a/2323034
     * 
     * @param angle any angle
     * @return if the angle is between the limits after normalized
     */
    public boolean isAngleGood(double angle) {
        angle = ((angle % 360) + 360) % 360;

        return angle > kMinRotationDeg && angle < kMaxRotationDeg;
    }

    public void setAngle(double angle) {
        // if (isAngleGood(angle)) {
        //     setPosition(angle);
        //     updatePositionFeedforward();
        // } else if (isAngleTooBig(angle)) {
        //     setPosition(kMaxRotationDeg);
        //     updatePositionFeedforward();
        // } else if (isAngleTooSmall(angle)) {
        //     setPosition(kMinRotationDeg);
        //     updatePositionFeedforward();
        // } else {
        //     end();
        // }
        setPosition(MathUtil.clamp(angle, kMinRotationDeg, kMaxRotationDeg));
        updatePositionFeedforward();
    }

    // public boolean isAngleTooBig(double angle) {
    //     angle = ((angle % 360) + 360) % 360;

    //     if (angle > 180) {
    //         angle -= 360;
    //     }
    //     return angle >= kMaxRotationDeg;
    // }

    // public boolean isAngleTooSmall(double angle) {
    //     angle = ((angle % 360) + 360) % 360;

    //     if (angle > 180) {
    //         angle -= 360;
    //     }
    //     return angle <= kMinRotationDeg;
    // }

    public void setAngleMotionMagic(double angle) {
        if (isAngleGood(angle)) {
            setPositionMotionMagic(angle);
        } else {
            end();
        }
    }

    public boolean isOnLimitSwitch() {
        return isOnLimitSwitch;
    }

    public double getAngle() {
        return getPosition();
    }

    @Override
    public String getName() {
        return "Turret";
    }

    public boolean isAiming() {
        return isAiming;
    }

    public void setAiming(boolean value) {
        isAiming = value;
    }
}