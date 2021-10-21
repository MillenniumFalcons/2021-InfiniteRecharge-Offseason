package team3647.frc2020.autonomous;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import team3647.frc2020.robot.Constants;
import team3647.frc2020.robot.Constants.cField;
import team3647.lib.util.Units;

public class Trajectories {

        private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
                        Constants.cDrivetrain.kA),
                Constants.cDrivetrain.kDriveKinematics, Constants.cDrivetrain.maxVoltage);

// Create config for trajectory
        private static final TrajectoryConfig forwardTrajectoryConfig =
        new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
                Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        private static final TrajectoryConfig reverseTrajectoryConfig =
        new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
                Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);

//SIX BALL BOTTOM (IN FRONT OF TOWER 3 SHOOT-TRENCH-SHOOT ) ////////////////////////////////////////////////////////////////////////////////////////
public static Trajectory initiationLineToTrenchBalls = TrajectoryGenerator.generateTrajectory(
        cField.startingPositionForTrenchRun, List.of(cField.trenchBall1, cField.trenchBall2),
        new Pose2d(cField.trenchBall3, cField.startingPositionForTrenchRun.getRotation()),
        reverseTrajectoryConfig);

public static Trajectory trenchBall3ToFrontOfTower = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cField.trenchBall3, new Rotation2d(0)), List.of(),
        new Pose2d(cField.initiationFrontOfTower, new Rotation2d(0)), forwardTrajectoryConfig);



//EIGHT BALL BOTTOM (IN FRONT OF TOWER 3 SHOOT-RENDEZVOUS-TRENCH-STOP)/////////////////////////////////////////////////////////////////////////////////////////
        public static Trajectory bumpersInFrontOfTowerToRendezvousBalls =
        TrajectoryGenerator.generateTrajectory(cField.robotInFrontOfTargetInitLine, List.of(),
                cField.getBallsFromRendezvousPosition, reverseTrajectoryConfig);

        public static Trajectory rendezvousEntryToGetRowOfBalls = 
        TrajectoryGenerator.generateTrajectory(cField.getBallsFromRendezvousPosition, List.of(cField.switchBottomBall2), 
        cField.switchTopBall1Pose, reverseTrajectoryConfig);

        public static Trajectory toMidPointOnWayToTrenchBalls =
        TrajectoryGenerator.generateTrajectory(cField.switchTopBall1Pose, List.of(),
                cField.poseBetweenRendezvousAndTrench, forwardTrajectoryConfig);

        public static Trajectory midPointToTrenchBalls = TrajectoryGenerator.generateTrajectory(
        cField.poseBetweenRendezvousAndTrench, List.of(cField.trenchBall1, cField.trenchBall2),
        new Pose2d(cField.trenchBall3, new Rotation2d(0)), reverseTrajectoryConfig);
        
//SOMETHING////////////////////////////////////////////////////////////////////////////////////////

        public static Trajectory initLineToOpponentTrenchBalls = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cField.pointOnInitLineInFrontOfOpponentTrench
                .minus(new Translation2d(cField.widthOfBumpers, 0)), new Rotation2d()),
        List.of(), new Pose2d(cField.pointOnOpponentTrenchForTwoBalls, new Rotation2d()),
        reverseTrajectoryConfig);

        public static Trajectory opponentTrenchBallsToShootingLocation =
        TrajectoryGenerator.generateTrajectory(
                new Pose2d(cField.pointOnOpponentTrenchForTwoBalls, new Rotation2d()),
                List.of(),
                new Pose2d(cField.pointToShootBallsFrom,
                        new Rotation2d(Units.degrees_to_radians(-90))),
                forwardTrajectoryConfig);

        public static Trajectory shootingPositionToRendezvous2Balls =
        TrajectoryGenerator.generateTrajectory(
                new Pose2d(cField.pointToShootBallsFrom,
                        new Rotation2d(Units.degrees_to_radians(-90))),
                List.of(), new Pose2d(cField.pointToIntakeTwoBallsFromRendezvous,
                        cField.angleOfRendezvousForThreeBalls),
                reverseTrajectoryConfig);

        public static Trajectory rendezvous2BallsToMidPoint = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cField.pointToIntakeTwoBallsFromRendezvous,
                cField.angleOfRendezvousForThreeBalls),
        List.of(), new Pose2d(cField.pointBeforeIntakeLastBallOnRendezvous,
                cField.angleOfRendezvousForThreeBalls),
        forwardTrajectoryConfig);

        public static Trajectory midPointToRendezvousLastBall = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cField.pointBeforeIntakeLastBallOnRendezvous,
                cField.angleOfRendezvousForThreeBalls),
        List.of(), new Pose2d(cField.pointForIntakeLastBallOnRendezvous,
                cField.angleOfRendezvousForThreeBalls),
        reverseTrajectoryConfig);

        public static Trajectory rendezvousLastBallToShootingPosition =
        TrajectoryGenerator.generateTrajectory(
                new Pose2d(cField.pointForIntakeLastBallOnRendezvous,
                        cField.angleOfRendezvousForThreeBalls),
                List.of(),
                new Pose2d(cField.pointToShootBallsFrom,
                        new Rotation2d(Units.degrees_to_radians(-90))),
                forwardTrajectoryConfig);

//Five BALL MIDDLE (MIDDLE 3 SHOOT-RENDEZVOUS TOP- RENDEVOUS BOTTOM- SHOOT) //////////////////////////////////////////////////////////////////////////////////////////////////

        public static Trajectory midTrajectBack =
                TrajectoryGenerator.generateTrajectory(Constants.cField.centerOnInit,
                List.of(
                Constants.cField.pollSequerePoint1,
                Constants.cField.switchTopBall2
                ),
                new Pose2d(cField.switchTopBall1, new Rotation2d(0)), reverseTrajectoryConfig);

        public static Trajectory midTrajectForward =
                TrajectoryGenerator.generateTrajectory(new Pose2d(cField.switchTopBall1, new Rotation2d(0)),
                List.of(cField.pollSequerePoint1),/*, Constants.cField.switchBottomBall2, Constants.cField.switchBottomBall3, Constants.cField.adjustmentPointBetweenSwitchInitLine*/
                cField.bottom1EndPose, forwardTrajectoryConfig);

/////
        public static Trajectory fiveBallHigh = 
                TrajectoryGenerator.generateTrajectory(cField.startingPosInfrontOfEnemyTrench, 
                List.of(cField.midPointToTrench),
                 new Pose2d(cField.twoTrenchBalls, new Rotation2d(0)), reverseTrajectoryConfig);
        //public static Trajectory trenchToShoot = 
                //TrajectoryGenerator.generateTrajectory(, interiorWaypoints, end, config)

<<<<<<< Updated upstream
=======
        //hide away 3 ball auto
        /*public static Trajectory threeBallLow = 
                TrajectoryGenerator.generateTrajectory(cField.startingPosDirectlyToShoot, 
                List.of(), new Pose2d(cField.hideAway, new Rotation2d(0)), forwardTrajectoryConfig);
*/
>>>>>>> Stashed changes
}
