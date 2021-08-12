/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

//.6 position 11ft 4"
public class InitiationLineShot extends ShootOpenloop {
    /**
     * Creates a new InitiationLineShot.
     */
    public InitiationLineShot(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, BallStopper ballStopper) {
        super(flywheel, kickerWheel, indexer, ballStopper, 3650, .5, 1.03, IndexerSignal.GO_FAST);
        // Use addRequirements() here to declare subsystem dependencies.
    }
}
