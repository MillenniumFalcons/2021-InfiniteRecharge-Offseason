package team3647.frc2020.autonomous;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import team3647.frc2020.robot.Constants;

public class Trajectories {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
            Constants.cDrivetrain.kA), Constants.cDrivetrain.kDriveKinematics, Constants.cDrivetrain.maxVoltage);     
    
    private static final TrajectoryConfig TrajectoryConfigForwards =
            new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
            Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.cDrivetrain.kDriveKinematics)
            .addConstraint(autoVoltageConstraint).setReversed(false);

    private static final TrajectoryConfig reverseTrajectoryConfig =
        new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
                Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);
    

    public static Trajectory GalaticSearch_A_RedTraject = TrajectoryGenerator.generateTrajectory(Constants.cField.GalaticSearch_A_Red_startingPoint,
    Arrays.asList(Constants.cField.GalaticSearch_A_Red_firstBall, Constants.cField.GalaticSearch_A_Red_secondBall, Constants.cField.GalaticSearch_A_Red_thirdBall),
     Constants.cField.GalaticSearch_A_Red_endingPoint, reverseTrajectoryConfig);

     public static Trajectory GalaticSearch_B_RedTraject = TrajectoryGenerator.generateTrajectory(Constants.cField.GalaticSearch_B_Red_startingPoint,
      Arrays.asList(Constants.cField.GalaticSearch_B_Red_firstBall, Constants.cField.GalaticSearch_B_Red_secondBall, Constants.cField.GalaticSearch_B_Red_thirdBall),
       Constants.cField.GalaticSearch_B_Red_endingPoint, reverseTrajectoryConfig);

       
       public static Trajectory AutoNav_BarrelRace = TrajectoryGenerator.generateTrajectory(Constants.cField.AutoNav_Barrel_Race_Start,
       Arrays.asList(Constants.cField.AUTONAV_Barrel_Race_1, Constants.cField.AUTONAV_Barrel_Race_2, Constants.cField.AUTONAV_Barrel_Race_3,
       Constants.cField.AUTONAV_Barrel_Race_4, Constants.cField.AUTONAV_Barrel_Race_5, Constants.cField.AUTONAV_Barrel_Race_6),
       Constants.cField.AutoNav_Barrel_Race_END, TrajectoryConfigForwards);

       public static Trajectory test = TrajectoryGenerator.generateTrajectory(Constants.cField.testStart,
       Arrays.asList(Constants.cField.test1), Constants.cField.testEnd, TrajectoryConfigForwards);


}
