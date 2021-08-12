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
public abstract class TimestampedData {
    private final double m_timestamp;

    public TimestampedData(double timestamp) {
        this.m_timestamp = timestamp;
    }

    /**
     * @return the m_timestamp in units entered
     */
    public double getTimestamp() {
        return m_timestamp;
    }
}
