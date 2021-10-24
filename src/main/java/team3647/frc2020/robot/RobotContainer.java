/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2020.autonomous.Trajectories;
import team3647.frc2020.commands.AccelerateFlywheelKickerWheel;
import team3647.frc2020.commands.ArcadeDrive;
import team3647.frc2020.commands.AutoAimTurretHood;
import team3647.frc2020.commands.BatterShot;
import team3647.frc2020.commands.BetterLoadingStation;
import team3647.frc2020.commands.DeployClimber;
import team3647.frc2020.commands.ExtendIntakeToGround;
import team3647.frc2020.commands.FlywheelOpenloop;
import team3647.frc2020.commands.GroundIntake;
import team3647.frc2020.commands.GroundIntakeSequence;
import team3647.frc2020.commands.IndexerManual;
import team3647.frc2020.commands.KickerWheelOpenloop;
import team3647.frc2020.commands.LoadBalls;
import team3647.frc2020.commands.LoadingStationIntake;
import team3647.frc2020.commands.MoveHood;
import team3647.frc2020.commands.OrganizeFeeder;
import team3647.frc2020.commands.RemoveBalls;
import team3647.frc2020.commands.ShootClosedLoop;
import team3647.frc2020.commands.StopShooting;
import team3647.frc2020.commands.StowIntakeAndOrganizeFeeder;
import team3647.frc2020.commands.StowIntakeCompletely;
import team3647.frc2020.commands.TrenchShot;
import team3647.frc2020.commands.TurretManual;
import team3647.frc2020.commands.TurretMotionMagic;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Climber;
import team3647.frc2020.subsystems.Drivetrain;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Hood;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.frc2020.subsystems.LED;
import team3647.frc2020.subsystems.Turret;
import team3647.frc2020.subsystems.VisionController;
import team3647.frc2020.subsystems.VisionController.Pipeline;
import team3647.frc2020.inputs.Joysticks;
import team3647.frc2020.inputs.Limelight.LEDMode;
import team3647.lib.DriveSignal;
import team3647.lib.GroupPrinter;
import team3647.lib.IndexerSignal;
import team3647.lib.wpi.Compressor;
import team3647.lib.wpi.PDP;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        private final Joysticks mainController = new Joysticks(0);
        private final Joysticks coController = new Joysticks(1);
        private final PDP pdp = new PDP();
        private final Compressor airCompressor = new Compressor(0);

        private final VisionController m_visionController = new VisionController(Constants.cVisionController.camIP,
                        Constants.cVisionController.camConstants);

        private final Drivetrain m_drivetrain = new Drivetrain(Constants.cDrivetrain.leftMasterConfig,
                        Constants.cDrivetrain.rightMasterConfig, Constants.cDrivetrain.leftSlaveConfig,
                        Constants.cDrivetrain.rightSlaveConfig, Constants.cDrivetrain.leftMasterPIDConfig,
                        Constants.cDrivetrain.rightMasterPIDConfig, Constants.cDrivetrain.shifterPin,
                        Constants.cDrivetrain.kWheelDiameter, Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
                        Constants.cDrivetrain.kA, 16, Constants.cDrivetrain.climbLimitPin);

        private final Flywheel m_flywheel = new Flywheel(Constants.cFlywheel.masterConfig,
                        Constants.cFlywheel.slaveConfig, Constants.cFlywheel.pidConfig);

        private final Indexer m_indexer = new Indexer(Constants.cIndexer.leftRollersConfig,
                        Constants.cIndexer.rightRollersConfig, Constants.cIndexer.tunnelConfig,
                        Constants.cIndexer.horizontalRollersConfig, Constants.cIndexer.bannerSensorPin,
                        pdp::getCurrent);

        private final Intake m_intake = new Intake(Constants.cIntake.intakeMotorConfig,
                        Constants.cIntake.innerPistonsPin, Constants.cIntake.outerPistonsPin);

        private final KickerWheel m_kickerWheel = new KickerWheel(Constants.cKickerWheel.masterConfig,
                        Constants.cKickerWheel.pidConfig);

        private final Turret m_turret = new Turret(Constants.cTurret.masterConfig, Constants.cTurret.pidConfig,
                        Constants.cTurret.kMaxRotationDeg, Constants.cTurret.kMinRotationDeg,
                        Constants.cTurret.limitSwitchPin);

        private final Hood m_hood = new Hood(Constants.cHood.pwmPort, Constants.cHood.minPosition,
                        Constants.cHood.maxPosition);

        private final Climber m_climber = new Climber(Constants.cClimber.solenoidPin);

        private final BallStopper m_ballStopper = new BallStopper(Constants.cBallStopper.solenoidPin);

        private final GroupPrinter m_printer = GroupPrinter.getInstance();

        private final LED m_LED = new LED(Constants.cLED.canifierPin);

        private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

        public RobotContainer() {
                airCompressor.start();
                pdp.clearStickyFaults();

                //MUST START IN STOWED FOR INTAKE TRANSITIONS TO WORK PROPERLY      
                m_intake.retractInner();
                m_intake.retractOuter();

                m_ballStopper.extend();
                m_commandScheduler.registerSubsystem(m_kickerWheel, m_flywheel, m_visionController, m_intake, m_indexer,
                                m_drivetrain, m_hood, m_ballStopper, m_printer, m_LED);

                m_commandScheduler.setDefaultCommand(m_drivetrain,
                                new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
                                                mainController::getRightStickX, mainController.rightJoyStickPress::get,
                                                mainController.buttonA::get));
                                                
                m_commandScheduler.setDefaultCommand(m_turret, new TurretManual(m_turret, coController::getLeftStickX));
                m_indexer.setDefaultCommand(new IndexerManual(m_indexer, coController::getRightStickY));

                m_LED.setDefaultCommand(new RunCommand(() -> {
                        boolean hasValidTarget = m_visionController.isValid();
                        boolean isAimed = hasValidTarget && Math.abs(m_visionController.getFilteredYaw()) < 1;
                        double targetAngleInForTurretPosition = m_turret.getAngle()
                                        - m_visionController.getFilteredYaw();
                        boolean isTargetOutsideLimits = hasValidTarget
                                        && !m_turret.isAngleGood(targetAngleInForTurretPosition);
                        boolean isTurretAiming = m_turret.isAiming();

                        if (isTurretAiming) {
                                if (isTargetOutsideLimits) {
                                        if (m_LED.getRed() > .8) {
                                                m_LED.set(0, 0, 0);
                                        } else {
                                                m_LED.set(1, 0, 0);
                                        }
                                } else if (hasValidTarget) {
                                        if (isAimed) {
                                                if (m_LED.getGreen() > .8) {
                                                        m_LED.set(0, 0, 0);
                                                } else {
                                                        m_LED.set(0, 1, 0);
                                                }
                                        } else {
                                                m_LED.set(0, 1, 0);
                                        }
                                } else {
                                        m_LED.set(1, 1, 0);
                                }
                        } else {
                                m_LED.set(1, 0, 0);
                        }
                }, m_LED));

                m_printer.addDouble("tunnel amps", () -> {
                        return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
                });
                m_printer.addDouble("funnel amps", () -> {
                        return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
                });

                m_printer.addDouble("shooter rpm based on distance", this::getFlywheelRPM);
                m_printer.addDouble("kicker wheel amps", m_kickerWheel::getMasterCurrent);
                m_printer.addDouble("hoodPosition", m_hood::getAppliedPosition);
                m_printer.addDouble("turret distance to target", m_visionController::getFilteredDistance);
                m_printer.addDouble("shooter velocity", m_flywheel::getVelocity);
                m_printer.addDouble("turret pos: ", m_turret::getAngle);
                m_printer.addDouble("gyro", m_drivetrain::getHeading);
                m_printer.addDouble("dt X", this::getDtX);
                m_printer.addDouble("dt Y", this::getDtY);
                
                m_printer.addDouble("dt velocity", m_drivetrain::getLeftVelocity);
                m_printer.addDouble("dt velocity", m_drivetrain::getRightVelocity);
                configButtonBindings();
        }

        private void configButtonBindings() {
                // coController.leftTrigger
                // .whenActive(new GroundIntakeSequence(m_intake, m_indexer, m_ballStopper));

                // coController.leftBumper.whenActive(new ParallelCommandGroup(
                // new LoadingStationIntake(m_intake), new LoadBalls(m_indexer,
                // m_ballStopper)));

                // One trigger ground intake
                coController.leftTrigger.whenPressed(new SequentialCommandGroup(
                                new RunCommand(m_intake::extendOuter, m_intake).withTimeout(0.2),
                                new RunCommand(m_intake::extendInner, m_intake).withTimeout(0.2)));

                // loading station
                coController.leftTrigger
                                .whenReleased(new RunCommand(m_intake::retractOuter, m_intake).withTimeout(0.3));


                coController.leftBumper.and(coController.leftTrigger.negate())
                                .whenActive(new SequentialCommandGroup(
                                                new RunCommand(m_intake::end, m_intake).withTimeout(0.2),
                                                new ExtendIntakeToGround(m_intake).withTimeout(0.4),
                                                new RunCommand(m_intake::retractOuter, m_intake).withTimeout(0.2)));

                coController.leftBumper.and(coController.leftTrigger).whenActive(new ParallelCommandGroup(
                                new GroundIntake(m_intake), new LoadBalls(m_indexer, m_ballStopper)));

                coController.leftBumper.whenReleased(new ParallelCommandGroup(new RunCommand(m_intake::end, m_intake), new RunCommand(m_indexer::end, m_indexer)));

                // coController.leftBumper
                // .whenReleased(new ParallelCommandGroup(new RunCommand(m_intake::end,
                // m_intake),
                // new OrganizeFeeder(m_indexer, m_kickerWheel).withTimeout(3)));

                coController.buttonA.whenActive(new SequentialCommandGroup(new RemoveBalls(m_indexer, m_intake, m_kickerWheel)));
                coController.buttonA.whenReleased(new SequentialCommandGroup(new RunCommand(() -> {
                        m_indexer.end();
                        m_kickerWheel.end();
                        m_intake.end();
                }, m_indexer, m_kickerWheel, m_intake).withTimeout(.5)));

                // climber deploy and get turret out of way
                mainController.buttonX.whenActive(new SequentialCommandGroup(
                                new TurretMotionMagic(m_turret, 180),
                                new DeployClimber(m_climber),
                                new TurretMotionMagic(m_turret, Constants.cTurret.backwardDeg).keepPosition()));

                coController.buttonX.whenActive(new ParallelCommandGroup(new FlywheelOpenloop(m_flywheel, -1),
                                new KickerWheelOpenloop(m_kickerWheel, -1)));

                coController.rightBumper.whenActive(new ParallelCommandGroup(
                new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition,
                m_visionController::getFilteredYaw, m_visionController::isValid),
                new AccelerateFlywheelKickerWheel(m_flywheel, m_kickerWheel,
                this::getFlywheelRPM,
                m_visionController::isValid)));
                // coController.rightBumper.whenActive(new RunCommand(m_ballStopper::extend, m_ballStopper));

                coController.leftMidButton.whenActive(new InstantCommand(() -> {
                        m_visionController.setPipeline(Pipeline.SUNNY_TARGETING);
                }));

                // coController.rightMidButton.whenActive(new InstantCommand(() -> {
                // m_visionController.setPipeline(Pipeline.CLOSE);
                // }));
                coController.rightMidButton.whenActive(new InstantCommand(() -> {
                        m_visionController.setPipeline(Pipeline.CLOSE);
                        }));



                coController.rightTrigger.whenActive(new ParallelCommandGroup(
                                new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition,
                                                m_visionController::getFilteredYaw, m_visionController::isValid),
                                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper,
                                                this::getFlywheelRPM,
                                                Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM,
                                                IndexerSignal.GO_FAST)));

                coController.leftJoyStickPress.whenActive(new TurretManual(m_turret, coController::getLeftStickX));
                coController.rightJoyStickPress.whenPressed(new StowIntakeCompletely(m_intake).withTimeout(.1));

                coController.buttonB.whenActive(
                                new ParallelCommandGroup(new MoveHood(m_hood, Constants.cHood.trenchShotPosition),
                                                new TrenchShot(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper)));

                coController.buttonY.whenActive(new ParallelCommandGroup(
                                new MoveHood(m_hood, Constants.cHood.rightUpToTowerShotPosition),
                                new BatterShot(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper)));

                coController.buttonY.or(coController.buttonB).or(coController.rightTrigger)
                                .whenActive(new RunCommand(airCompressor::stop).withTimeout(.1));
                coController.buttonY.or(coController.buttonB).or(coController.rightTrigger)
                                .whenInactive(new RunCommand(airCompressor::start).withTimeout(.1));
                // coController.rightTrigger.whenActive(
                // new TrenchShot(m_flywheel, m_kickerWheel, m_indexer)
                // );

                coController.rightTrigger.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

                coController.buttonB.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

                coController.buttonY.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));
                coController.buttonX.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

                coController.dPadLeft
                                .whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.leftDeg).keepPosition());
                // coController.dPadRight.whenPressed(
                // new TurretMotionMagic(m_turret, Constants.cTurret.rightDeg).withTimeout(.5));
                coController.dPadUp.whenPressed(
                                new TurretMotionMagic(m_turret, Constants.cTurret.forwardDeg).keepPosition());
                coController.dPadDown.whenPressed(
                                new TurretMotionMagic(m_turret, Constants.cTurret.backwardDeg).keepPosition());

                coController.dPadRight.whenPressed(new MoveHood(m_hood, .645));

        }

        public void init() {
                m_drivetrain.init();
                //SIX BALL BOTTOM AUTO
                m_drivetrain.setOdometry(Constants.cField.startingPositionForTrenchRun, new Rotation2d(0));
                //EIGHT BALL MIDDLE AUTO
                //m_drivetrain.setOdometry(Constants.cField.centerOnInit, new Rotation2d(0));
                m_flywheel.init();
                m_indexer.init();
                m_intake.init();
                m_kickerWheel.init();
                m_turret.init();
                m_visionController.init();
                m_visionController.setLedMode(LEDMode.ON);
                m_visionController.setPipeline(Pipeline.OUTSIDE_CLOUDY);
                m_turret.setAngle(0);
        }

        public void onDisabled() {
                m_visionController.setLedMode(LEDMode.OFF);
                m_intake.retractOuter();
                m_intake.retractInner();
        }


        
        
//SIX BALL BOTTOM (IN FRONT OF TOWER 3 SHOOT-TRENCH-SHOOT ) ////////////////////////////////////////////////////////////////////////////////////////
        private final RamseteCommand initiationLineToTrench =
                new RamseteCommand(Trajectories.initiationLineToTrenchBalls, m_drivetrain::getPose,
                        new RamseteController(), Constants.cDrivetrain.kDriveKinematics,
                        m_drivetrain::setVelocityMpS, m_drivetrain);
        private final RamseteCommand trenchToCenterTower = new RamseteCommand(
                Trajectories.trenchBall3ToFrontOfTower, m_drivetrain::getPose, new RamseteController(),
                Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS, m_drivetrain);
//EIGHT BALL BOTTOM (IN FRONT OF TOWER 3 SHOOT-RENDEZVOUS-TRENCH-STOP)/////////////////////////////////////////////////////////////////////////////////////////
        private final RamseteCommand initiationLineToRendezvousBalls = new RamseteCommand(
                Trajectories.bumpersInFrontOfTowerToRendezvousBalls, m_drivetrain::getPose, new RamseteController(), 
                Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS, m_drivetrain);

//EIGHT BALL MIDDLE (MIDDLE 3 SHOOT-RENDEZVOUS TOP- RENDEVOUS BOTTOM- SHOOT) //////////////////////////////////////////////////////////////////////////////////////////////////
        private final RamseteCommand midFiveBack = 
                        new RamseteCommand(Trajectories.midTrajectBack, m_drivetrain::getPose, new RamseteController(), 
                        Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS, m_drivetrain);
        private final RamseteCommand midFiveForward = 
                        new RamseteCommand(Trajectories.midTrajectForward, m_drivetrain::getPose, new RamseteController(), 
        Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS, m_drivetrain);

        private final Command sixBallBottom =  new SequentialCommandGroup(
                new TurretMotionMagic(m_turret, 35).withTimeout(.6),
                new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition,
                        m_visionController::getFilteredYaw, m_visionController::isValid)
                                .withTimeout(.3),
                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper,
                        this::getFlywheelRPM, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM,
                        IndexerSignal.GO_FAST).withTimeout(3),
                new StopShooting(m_flywheel, m_kickerWheel, m_indexer),
                new ParallelDeadlineGroup(initiationLineToTrench,
                        new GroundIntakeSequence(m_intake, m_indexer, m_ballStopper)),
                new RunCommand(this::stopDrivetrain, m_drivetrain).withTimeout(.1),
                new ParallelDeadlineGroup(
                        trenchToCenterTower,
                        new SequentialCommandGroup(
                                new StowIntakeAndOrganizeFeeder(m_intake, m_indexer, m_kickerWheel)
                                        .withTimeout(2),
                                new LoadBalls(m_indexer, m_ballStopper)),
                        new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition,
                                m_visionController::getFilteredYaw, m_visionController::isValid),
                        new AccelerateFlywheelKickerWheel(m_flywheel, m_kickerWheel, 4000, true)),
                new RunCommand(this::stopDrivetrain, m_drivetrain).withTimeout(.1),
                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper,
                        this::getFlywheelRPM, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM,
                        IndexerSignal.GO_FAST).withTimeout(4),
                new StopShooting(m_flywheel, m_kickerWheel, m_indexer));
        //MID FIVE////////////////////////////

        private final Command midFive = new SequentialCommandGroup(
        new ParallelDeadlineGroup(midFiveBack, new GroundIntakeSequence(m_intake, m_indexer, m_ballStopper)),midFiveForward,
        new StowIntakeAndOrganizeFeeder(m_intake, m_indexer, m_kickerWheel)
                                .withTimeout(2),
                        new LoadBalls(m_indexer, m_ballStopper),
                new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition,
                        m_visionController::getFilteredYaw, m_visionController::isValid),
                        new AccelerateFlywheelKickerWheel(m_flywheel, m_kickerWheel, 4050, true),
                        new RunCommand(this::stopDrivetrain, m_drivetrain).withTimeout(.1),
                        new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper,
                                this::getFlywheelRPM, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM,
                                IndexerSignal.GO_FAST).withTimeout(4),
                        new StopShooting(m_flywheel, m_kickerWheel, m_indexer));
        
        // private final Command testPath = new SequentialCommandGroup(
        //         middle8BallsPath,  
        //         new RunCommand(this::stopDrivetrain, m_drivetrain).withTimeout(.1));
                
        
                                
        public Command getAutonomousCommand() {

                return sixBallBottom;
        }       

        public void stopDrivetrain() {
                m_drivetrain.setOpenLoop(DriveSignal.BRAKE);
        }

        private double getHoodPosition() {
                return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        }

        private double getFlywheelRPM() {
                return Constants.cFlywheel.getFlywheelRPM(m_visionController.getFilteredDistance());
        }

        private double getDtX() {
                return m_drivetrain.getPose().getX();
        }

        private double getDtY() {
                return m_drivetrain.getPose().getY();
        }

}
