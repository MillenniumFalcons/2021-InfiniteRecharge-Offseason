/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Pose2d with a timestamp
 */
public class TimeStampedRobotPose extends TimestampedData {
    private final Pose2d m_pose;

    public TimeStampedRobotPose(double timestamp, Pose2d pose) {
        super(timestamp);
        this.m_pose = pose;
    }

    public Pose2d getPose() {
        return m_pose;
    }
}
