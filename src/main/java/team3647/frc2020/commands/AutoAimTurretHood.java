/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team3647.frc2020.subsystems.Turret;
import team3647.frc2020.subsystems.Hood;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAimTurretHood extends ParallelCommandGroup {
    /**
     * Creates a new AutoAimAccelerateFlywheel.
     */
    public AutoAimTurretHood(Hood hood, Turret turret, DoubleSupplier hoodPosition,
            DoubleSupplier angleToTurret, BooleanSupplier hasValidTarget) {
        super(new AimTurret(turret, angleToTurret, hasValidTarget), new MoveHood(hood, hoodPosition, hasValidTarget));
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
    }
}
