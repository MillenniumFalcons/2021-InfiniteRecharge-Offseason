/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import team3647.frc2020.robot.Constants;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public class TrenchShot extends ShootClosedLoop {
    /**
     * Creates a new TrenchShot.
     */
    public TrenchShot(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, BallStopper ballStopper) {
        super(flywheel, kickerWheel, indexer, ballStopper, () -> {
            return 5700;
        }, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM, IndexerSignal.GO);
        // Use addRequirements() here to declare subsystem dependencies.
    }
}
