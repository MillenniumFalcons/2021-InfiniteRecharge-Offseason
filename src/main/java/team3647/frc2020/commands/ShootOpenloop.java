/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public abstract class ShootOpenloop extends CommandBase {
    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final BallStopper m_ballStopper;
    private boolean reachedVelOnce;
    private double motorOutputAtRPM = 1;

    private final double shooterRPM;
    private final double kickerWheelOutput;
    private final double percentIncrease;
    private final IndexerSignal indexerSignal;

    // private final Roll
    /**
     * Shoot balls continuously while execute is ran
     */
    public ShootOpenloop(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, BallStopper ballStopper,
            double shooterRPM, double kickerWheelOutput, double percentIncrease, IndexerSignal indexerSignal) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        m_ballStopper = ballStopper;
        addRequirements(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper);

        this.shooterRPM = shooterRPM;
        this.kickerWheelOutput = kickerWheelOutput;
        this.percentIncrease = percentIncrease;
        this.indexerSignal = indexerSignal;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        reachedVelOnce = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // AUTO SHOT
        m_kickerWheel.setOpenloop(kickerWheelOutput);

        if (m_flywheel.getVelocity() > shooterRPM) {
            motorOutputAtRPM = m_flywheel.getOutput();

            reachedVelOnce = true;

        } else if (!reachedVelOnce) {
            m_flywheel.setRPM(shooterRPM);
        }

        if (reachedVelOnce) {
            m_ballStopper.retract();
            m_indexer.set(indexerSignal);
            m_flywheel.setOpenloop(motorOutputAtRPM * percentIncrease);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_flywheel.end();
        m_kickerWheel.end();
        m_indexer.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // AUTO SHOT
    // m_kickerWheel.setOpenloop(.35);

    // if (m_flywheel.getVelocity() > 5500) {
    // motorOutputAtRPM = m_flywheel.getOutput();
    // reachedVelOnce = true;

    // } else if (!reachedVelOnce) {
    // m_flywheel.setRPM(5500);
    // }

    // if (reachedVelOnce) {
    // m_indexer.set(IndexerSignal.GO_FAST);
    // m_flywheel.setOpenloop(motorOutputAtRPM * 1.02);
    // }
    // ------------------------------------------------------------------
    // initiation line : (2 rollers #4) battery 12.6 disabled

    // m_kickerWheel.setOpenloop(.35);

    // if (m_flywheel.getVelocity() > 4300) {
    // motorOutputAtRPM = m_flywheel.getOutput();
    // reachedVelOnce = true;

    // } else if (!reachedVelOnce) {
    // m_flywheel.setRPM(4300);
    // }

    // if (reachedVelOnce) {
    // m_indexer.set(IndexerSignal.GO_FAST);
    // m_flywheel.setOpenloop(motorOutputAtRPM * 1.03);
    // }
    // ------------------------------------------------------------------

}
