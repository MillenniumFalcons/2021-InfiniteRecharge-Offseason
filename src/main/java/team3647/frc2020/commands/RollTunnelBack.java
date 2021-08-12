/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public class RollTunnelBack extends CommandBase {

    private final Indexer m_indexer;
    private final KickerWheel m_kickerWheel;
  /**
   * Creates a new RollIndexerBack.
   */
  public RollTunnelBack(Indexer indexer, KickerWheel kickerWheel) {
      m_indexer = indexer;
      m_kickerWheel = kickerWheel;
      addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_indexer.set(IndexerSignal.TUNNELDOWN_HOTDOGOUT);
      m_kickerWheel.setOpenloop(-.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_indexer.set(IndexerSignal.STOP);
      m_kickerWheel.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_indexer.getBannerSensorValue();
  }
}
