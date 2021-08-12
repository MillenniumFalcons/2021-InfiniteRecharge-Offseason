/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public class ShootClosedLoop extends CommandBase {
    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final BallStopper m_ballStopper;
    private final DoubleSupplier shooterRPM;
    private final Function<Double, Double> kickerWheelOutput;
    private final IndexerSignal signalOnShoot;

    /**
     * Creates a new ShootClosedLoop.
     */
    public ShootClosedLoop(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, BallStopper ballStopper,
            DoubleSupplier shooterRPM, Function<Double, Double> kickerWheelOutput, IndexerSignal signalOnShoot) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        m_ballStopper = ballStopper;

        this.kickerWheelOutput = kickerWheelOutput;
        this.shooterRPM = shooterRPM;
        this.signalOnShoot = signalOnShoot;
        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_ballStopper.retract();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_kickerWheel.setOpenloop(kickerWheelOutput.apply(shooterRPM.getAsDouble()));
        System.out.println("kicker wheel output: " + kickerWheelOutput.apply(shooterRPM.getAsDouble()));
        m_ballStopper.retract();
        m_flywheel.setRPM(shooterRPM.getAsDouble());

        if (m_flywheel.reachedTargetVelocity()) {
            if (m_indexer.getBannerSensorValue()) {
                m_indexer.set(signalOnShoot);
            } else {
                m_indexer.set(signalOnShoot);
            }
        } else {
            if (m_indexer.getBannerSensorValue()) {
                m_indexer.set(IndexerSignal.TUNNELHOLD_GO);
            } else {
                m_indexer.set(IndexerSignal.GO_SLOW);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
