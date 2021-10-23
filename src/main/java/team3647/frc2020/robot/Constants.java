package team3647.frc2020.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpiutil.math.MathUtil;
import team3647.lib.util.Units;
import team3647.frc2020.subsystems.VisionController;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;
import team3647.lib.util.InterpolatingDouble;
import team3647.lib.util.InterpolatingTreeMap;
import team3647.lib.util.RGB;

public class Constants {

    public static class cDrivetrain {
        // CAN id
        public static final int leftMasterPin = 1;
        public static final int leftSlavePin = 2;
        public static final int rightMasterPin = 3;
        public static final int rightSlavePin = 4;
        public static final int stallCurrent = 35;
        public static final int maxCurrent = 60;

        public static final int climbLimitPin = 0;

        // pcm pin
        public static final int shifterPin = 2;

        // meters (6inches)
        public static final double kWheelDiameter = .1524;

        // volts
        public static final double kS = 0.214;
        public static final double kV = 2.62;
        public static final double kA = 0.42;
        // public static final double kA = 0.15;

        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double onBoardkP = 14.8;

        public static final double kTrackwidthMeters = 0.6455447508046545;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double maxVoltage = 11.0;
        public static final double gearboxReduction = 9.0 / 42.0 * 24.0 / 50.0;

        public static final double neoRotationsToMeters =
                gearboxReduction * kWheelDiameter * Math.PI;

        public static final SparkMaxFactory.Configuration leftMasterConfig =
                new SparkMaxFactory.Configuration(leftMasterPin, false)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration rightMasterConfig =
                new SparkMaxFactory.Configuration(rightMasterPin, true)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration leftSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(leftMasterConfig, leftSlavePin);

        public static final SparkMaxFactory.Configuration rightSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(rightMasterConfig, rightSlavePin);

        public static final ClosedLoopConfig leftMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);
        public static final ClosedLoopConfig rightMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);

    }

    public static class cIntake {
        public static final int outerPistonsPin = 4;
        public static final int innerPistonsPin = 3;
        public static final int intakeMotorPin = 8;

        public static final boolean inverted = true;
        public static TalonSRXFactory.Configuration intakeMotorConfig =
                new TalonSRXFactory.Configuration(intakeMotorPin, inverted)
                        .configOpenLoopRampRate(.3);
    }

    public static class cPPSpinner {
        public static final RGB red = new RGB(new double[] {.51, .35, .14});
        public static final RGB green = new RGB(new double[] {.15, .59, .25});
        public static final RGB blue = new RGB(new double[] {.12, .42, .45});
        public static final RGB yellow = new RGB(new double[] {.32, .56, .12});
        // public static final RGB test = new RGB(new double[] {});
        public static final double colorThreshold = 0.05; // percentage
    }

    public static class cVisionController {
        // 8ft in meters
        public static final double kGoalHeight = Units.inches_to_meters(98.25);

        // 30inches in meters
        public static final double kCameraHeight = Units.feet_to_meters(3);

        // can be either 75 or 56 degrees depending on the lens setting used
        public static final double kFOV = 75;

        /** in micro meters */
        public static final double kSensorHeight = 2952;
        /** in micro meters */
        public static final double kSensorWidth = 3984;

        /** micrometers per pixel */
        public static final double kPixelSize = 6;

        /** in degrees */
        public static final double camAngle = 23.3;

        public static final double kImageCaptureLatency = 11.0 / 1000.0; // miliseconds

        public static final String camIP = "10.36.47.15";

        public static final VisionController.CamConstants camConstants =
                new VisionController.CamConstants(kGoalHeight, kCameraHeight, camAngle,
                        kImageCaptureLatency);
    }

    public static class cTurret {
        public static final int masterPin = 17;
        public static final boolean inverted = true;
        public static final double nominalVoltage = 10.0;
        public static final int limitSwitchPin = 6;
        public static final boolean sensorInverted = false;

        public static final double kMinRotationDeg = -20;
        public static final double kMaxRotationDeg = 200;

        public static final double kCruiseVelocityRPM = 150;
        public static final double kAccelerationRPMs = 300;
        // amps
        public static final int peakCurrent = 40;
        // milis
        public static final int peakCurrentDuration = 1000;
        // amps
        public static final int continuousCurrent = 20;

        public static final double reductionFromEncoder = 16.0 / 130.0;

        public static final double encoderTicksToUnits =
                (reductionFromEncoder / magEncoderTicksPerRev) * 360.0;
        public static final double encoderVelocityToRPM = encoderTicksToUnits / 360.0 * 10 * 60;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 30;

        public static final double kS = 1.2;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double forwardDeg = 0;
        public static final double backwardDeg = 180;
        public static final double leftDeg = 90;
        public static final double rightDeg = -90;

        public static final TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);

        public static final ClosedLoopConfig pidConfig = new ClosedLoopConfig()
                .encoderTicksToUnits(encoderTicksToUnits).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM).positionThreshold(.5)
                .maxVelocity(50).maxAcceleration(kAccelerationRPMs).sensorInverted(sensorInverted)
                .configPID(kP, kI, kD).configFeedForward(kS, kV, kA);
    }

    public static class cIndexer {
        public static final int leftVerticalRollersPin = 22;
        public static final int rightVerticalRollersPin = 21;
        public static final int tunnelPin = 23;
        public static final int horizontalRollersPin = 24;
        public static final int bannerSensorPin = 1;

        public static final int PP_VerticalPDPSlot = 10;
        public static final int tunnelPDPSlot = 8;

        public static boolean rightVerticalRollersInverted = false;
        public static boolean leftVerticalRollersInverted = true;
        public static boolean tunnelInverted = false;
        public static boolean horizontalRollersInverted = false;

        public static VictorSPXFactory.Configuration leftRollersConfig =
                new VictorSPXFactory.Configuration(leftVerticalRollersPin)
                        .setInverted(leftVerticalRollersInverted).configOpenLoopRampRate(.3)
                        .setPDPSlot(10);
        public static TalonSRXFactory.Configuration rightRollersConfig =
                new TalonSRXFactory.Configuration(rightVerticalRollersPin,
                        rightVerticalRollersInverted);
        public static VictorSPXFactory.Configuration tunnelConfig =
                new VictorSPXFactory.Configuration(tunnelPin).setInverted(tunnelInverted)
                        .setPDPSlot(8);
        public static VictorSPXFactory.Configuration horizontalRollersConfig =
                new VictorSPXFactory.Configuration(horizontalRollersPin)
                        .setInverted(horizontalRollersInverted).configOpenLoopRampRate(.3);

    }

    public static class cKickerWheel {
        public static final int masterPin = 15;
        public static final boolean inverted = false;
        public static final int peakCurrent = 60;
        public static final int peakCurrentDuration = 1000;
        public static final int continuousCurrent = 20;
        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double encoderTicksToUnits = 1 / magEncoderTicksPerRev;
        public static final double encoderVelocityToRPM = encoderTicksToUnits * 10 * 60;

        public static final double visionDistanceToRPM = 1;

        public static TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .currentLimiting(true, 20, 1, 10).voltageCompensation(true, 12);
        // .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
        // .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);
        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD);
        // .configFeedForward(kS, kV, kA).encoderTicksToUnits(encoderTicksToUnits)
        // .encoderVelocityToRPM(encoderVelocityToRPM)
        // .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double getFlywheelOutputFromFlywheelRPM(double rpm) {
            // return MathUtil.clamp(rpm / 2 * 0.000111111, .5, 1);
            return .9;
        }
    }

    public static class cFlywheel {
        public static final int masterPin = 11;
        public static final int slavePin = 12;
        public static final boolean masterInverted = false;
        public static final boolean slaveInverted = false;

        public static final int maxFreeSpeedCurrent = 60;
        public static final int maxStallCurrent = 40;

        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0.0005;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.623;
        public static final double kV = 0.0631;
        // public static final double kV = 0.03;
        public static final double kA = 0.0241;

        public static final double compkS = 0.393;
        public static final double compkV = 0.0635;
        public static final double compkA = 0.0215;

        public static final double kGearboxReduction = 2;
        public static final double encoderTicksToUnits = 42;
        public static final double encoderVelocityToRPM = 2;

        public static final double visionDistanceToRPM = 1;

        public static SparkMaxFactory.Configuration masterConfig =
                new SparkMaxFactory.Configuration(masterPin, masterInverted)
                        .idleMode(IdleMode.kBrake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, maxFreeSpeedCurrent, maxStallCurrent);
        public static SparkMaxFactory.Configuration slaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(masterConfig, slavePin);

        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD)
                .configFeedForward(compkS, compkV, compkA).encoderTicksToUnits(encoderTicksToUnits)
                .velocityThrehsold(125).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double calculateRPM(double distance) {
            return distance;
        }

        //when distance is ___ fly wheel RPM is ___
        public static double[][] kFlywheelDistanceRPM =
                {{Units.feet_to_meters(4), 2500}, {Units.feet_to_meters(6), 3000},
                        {Units.feet_to_meters(8), 3500}, {Units.feet_to_meters(10), 4000},
                        {Units.feet_to_meters(12), 4500}, {Units.feet_to_meters(14), 4800},
                        {Units.feet_to_meters(16), 4900}, {Units.feet_to_meters(18), 5300},
                        {Units.feet_to_meters(20), 5500}, {Units.feet_to_meters(22), 5700}};

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap =
                new InterpolatingTreeMap<>();

        static {
            for (double[] pair : kFlywheelDistanceRPM) {
                kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]),
                        new InterpolatingDouble(pair[1]));
            }
        }

        public static double getFlywheelRPM(double range) {
            InterpolatingDouble d =
                    kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range));

            return d == null ? 6000 : MathUtil.clamp(d.value, 2000, 8000);
        }
    }

    public static class cHood {
        public static int pwmPort = 2;
        public static double minPosition = .3;
        public static double maxPosition = .9;
        public static double trenchShotPosition = .645;
        public static double cpShotPosition = .66;
        public static double rightUpToTowerShotPosition = .25;

        public static double[][] kHoodDistancePosition =
                {{Units.feet_to_meters(4), .31}, {Units.feet_to_meters(6), .43},
                        {Units.feet_to_meters(8), .52}, {Units.feet_to_meters(10), .55},
                        {Units.feet_to_meters(12), .575}, {Units.feet_to_meters(14), .60},
                        {Units.feet_to_meters(16), .605}, {Units.feet_to_meters(18), .62},
                        {Units.feet_to_meters(20), .63}, {Units.meters_to_feet(22), .635}};

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap =
                new InterpolatingTreeMap<>();
        static {
            for (double[] pair : kHoodDistancePosition) {
                kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]),
                        new InterpolatingDouble(pair[1]));
            }
        }

        public static double getHoodPosition(double range) {
            InterpolatingDouble d = kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));
            return d == null ? .61 : MathUtil.clamp(d.value, .2, .7);
        }
    }

    public static class cLED {
        public static final int canifierPin = 0;
    }

    public static class cBallStopper {
        public static final int solenoidPin = 6;
    }

    public static class cField {
        public static final double metersTrenchBallsFromWall = Units.inches_to_meters(27.25);
        public static final double metersOpponentTrenchBallsFromWall = Units.inches_to_meters(296);
        public static final double metersInitiationLineFromDriversStation =
                Units.inches_to_meters(509.25);
        public static final double widthOfBumpers = Units.inches_to_meters(3.2);
        public static final double lengthOfRobot = Units.inches_to_meters(29);
        public static final double metersCenterOfTowerFromSide = Units.inches_to_meters(94.665);


        public static final Translation2d trenchBall1 =
                new Translation2d(Units.inches_to_meters(386.62), metersTrenchBallsFromWall);
        public static final Translation2d trenchBall2 =
                new Translation2d(Units.inches_to_meters(350.62), metersTrenchBallsFromWall);
        public static final Translation2d trenchBall3 =
                new Translation2d(Units.inches_to_meters(314.62), metersTrenchBallsFromWall);
        
        //left to right
        public static final Translation2d switchTopBall1 = 
                new Translation2d(Units.inches_to_meters(358.87), Units.inches_to_meters(159.385));
        public static final Translation2d switchTopBall2 = 
                new Translation2d(Units.inches_to_meters(377.34), Units.inches_to_meters(167.045));

        public static final Translation2d switchBottomBall1 = 
                new Translation2d(Units.inches_to_meters(352.56), Units.inches_to_meters(122.345));
        public static final Translation2d switchBottomBall2 = 
                new Translation2d(Units.inches_to_meters(371.04), Units.inches_to_meters(129.995));
        public static final Translation2d switchBottomBall3 = 
                new Translation2d(Units.inches_to_meters(389.52), Units.inches_to_meters(137.655));

        
        public static final Pose2d switchTopBall1Pose = 
                new Pose2d(switchTopBall1, new Rotation2d(Units.degrees_to_radians(-90)));

        public static final Translation2d pollSequerePoint1 = 
                new Translation2d(Units.inches_to_meters(400), Units.inches_to_meters(180));

        public static final Translation2d farTestPoint1 = 
                new Translation2d(Units.inches_to_meters(330), Units.inches_to_meters(148));

        public static final Translation2d farTestPoint2 = 
                new Translation2d(Units.inches_to_meters(310), Units.inches_to_meters(150));

        public static final Translation2d farTestPoint3 = 
                new Translation2d(Units.inches_to_meters(300), Units.inches_to_meters(130));

        public static final Translation2d farTestPoint4 = 
                new Translation2d(Units.inches_to_meters(320), Units.inches_to_meters(110));

        public static final Translation2d pollSequerePoin2 = 
                new Translation2d(Units.inches_to_meters(400), Units.inches_to_meters(90));

//SIX BALL BOTTOM//////////////////////////////////////////////////////////////
        public static final Pose2d startingPositionForTrenchRun =
                new Pose2d(metersInitiationLineFromDriversStation, metersTrenchBallsFromWall,
                        new Rotation2d(Units.degrees_to_radians(0)));

        public static final Translation2d initiationFrontOfTower = new Translation2d(
                metersInitiationLineFromDriversStation, metersCenterOfTowerFromSide);
        public static final Translation2d bumpersOnInitiationFrontOfTower = initiationFrontOfTower
                .minus(new Translation2d(lengthOfRobot / 2 + widthOfBumpers, 0));
        public static final Translation2d pointInFrontOfBallsInRendezvous =
                new Translation2d(Units.inches_to_meters(389.9), Units.inches_to_meters(109.811));
        public static final Translation2d pointBetweenRendezvousAndTrench =
                new Translation2d(Units.inches_to_meters(444), Units.inches_to_meters(84));
        public static final Pose2d robotInFrontOfTargetInitLine =
                new Pose2d(bumpersOnInitiationFrontOfTower, new Rotation2d(0));
        public static final Pose2d getBallsFromRendezvousPosition = new Pose2d(
                pointInFrontOfBallsInRendezvous, new Rotation2d(Units.degrees_to_radians(-67.5)));
        public static final Pose2d poseBetweenRendezvousAndTrench = new Pose2d(
                pointBetweenRendezvousAndTrench, new Rotation2d(Units.degrees_to_radians(45)));

// **8 ball from opponent trench, NOT adjusted for bumpers*//////////////////
        public static final Translation2d pointOnInitLineInFrontOfOpponentTrench =
                new Translation2d(metersInitiationLineFromDriversStation,
                        metersOpponentTrenchBallsFromWall);
        public static final Translation2d pointOnOpponentTrenchForTwoBalls =
                new Translation2d(Units.feet_to_meters(34.275), metersOpponentTrenchBallsFromWall);
        public static final Translation2d pointOnInitLineToGetRendezvousBalls = new Translation2d(
                metersInitiationLineFromDriversStation, Units.feet_to_meters(11.536));
        public static final Translation2d pointToShootBallsFrom =
                new Translation2d(Units.feet_to_meters(37.963), Units.feet_to_meters(12.822));
        public static final Translation2d pointToIntakeTwoBallsFromRendezvous =
                new Translation2d(Units.feet_to_meters(34.837), Units.feet_to_meters(14.614));
        public static final Translation2d pointBeforeIntakeLastBallOnRendezvous =
                new Translation2d(Units.feet_to_meters(37.927), Units.feet_to_meters(13.419));
        public static final Translation2d pointForIntakeLastBallOnRendezvous =
                new Translation2d(Units.feet_to_meters(35.655), Units.feet_to_meters(12.524));

        /**
         * the one perpendicular to the generator switch
         */
        public static final Rotation2d angleOfRendezvousForThreeBalls =
                new Rotation2d(Units.degrees_to_radians(22.5));
        public static final Rotation2d angleToShootLast3Balls =
                new Rotation2d(Units.degrees_to_radians(-24));
        /**
         * the one perpendicular to the generator switch
         */
        
//MIDDLE 5 BALLS
        public static final Pose2d centerOnInit =
                new Pose2d(Units.inches_to_meters(509.25), Units.inches_to_meters(161.625), new Rotation2d(0));

        public static final Translation2d adjustmentPointBetweenSwitchInitLine  =
                new Translation2d(Units.feet_to_meters(450), Units.feet_to_meters(150)); 
        
        public static final Pose2d endPositionForMiddle8Ball = 
                new Pose2d(Units.inches_to_meters(509.25), Units.inches_to_meters(94.57), new Rotation2d(0));
        // transperent
        public static final Translation2d topToBottomTransRendevousPoint = 
                new Translation2d(Units.inches_to_meters(336),Units.inches_to_meters(151));

                
        public static final Pose2d topToBottomTransRendevousPose = 
        new Pose2d(Units.inches_to_meters(336),Units.inches_to_meters(151), new Rotation2d(Units.degrees_to_radians(0)));

        public static final Pose2d bottom1EndPose =
        new Pose2d(Units.inches_to_meters(480), Units.inches_to_meters(100), new Rotation2d(0));

// Top Auto Points
        public static final Pose2d startingPosInfrontLoading =
                new Pose2d(Units.inches_to_meters(120), Units.inches_to_meters(100), new Rotation2d(0));
        public static final Translation2d topFirstTranslation1 =
                new Translation2d(Units.inches_to_meters(220), Units.inches_to_meters(90));
        public static final Pose2d bySydeToControlPanel =
                new Pose2d(Units.inches_to_meters(250), Units.inches_to_meters(32), new Rotation2d(Units.degrees_to_radians(-90)));
        
        
        public static final Pose2d infrontOfTargetParallelToControlPanel = 
                new Pose2d(Units.inches_to_meters(150), Units.inches_to_meters(230), new Rotation2d(Units.degrees_to_radians(0)));

//Straight Auto Points
        public static final Pose2d startingPoseInfrontOfPoleOnInit = 
                new Pose2d(Units.inches_to_meters(509.25), Units.inches_to_meters(126.879), new Rotation2d(0));
        public static final Pose2d endingPoseBumpersTouchWall = 
                new Pose2d(Units.inches_to_meters(605), Units.inches_to_meters(126.879), new Rotation2d(0));
    }

    public static class cClimber {
        public static int solenoidPin = 7;
    }

    /**
     * In order to make the class not be able to be an object
     */
    private Constants() {
    }

    public static final double magEncoderTicksPerRev = 4096.0;
}