/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.util;

/**
 * Add your docs here.
 */
public class TimestampedTurretAngle extends TimestampedData {
    private final double m_angle;

    public TimestampedTurretAngle(double timestamp, double angle) {
        super(timestamp);
        m_angle = angle;
    }

    /**
     * @return the m_angle
     */
    public double getAngle() {
        return m_angle;
    }
}
