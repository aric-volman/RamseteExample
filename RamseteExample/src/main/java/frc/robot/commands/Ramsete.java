// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Ramsete extends SequentialCommandGroup {
  /** Creates a new Ramsete. */
  public Ramsete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.RamseteConstants.kS,
        Constants.RamseteConstants.kV,
        Constants.RamseteConstants.kA),
        Constants.RamseteConstants.kDriveKinematics,
    10);

    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.RamseteConstants.kMaxSpeed,
      Constants.RamseteConstants.kMaxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.RamseteConstants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory newTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            newTrajectory,
            RobotContainer.dt::getPose,
            new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.RamseteConstants.kS,
              Constants.RamseteConstants.kV,
              Constants.RamseteConstants.kA),
              Constants.RamseteConstants.kDriveKinematics,
            RobotContainer.dt::getWheelSpeeds,
            new PIDController(Constants.RamseteConstants.kVel, 0, 0),
            new PIDController(Constants.RamseteConstants.kVel, 0, 0),
            // RamseteCommand passes volts to the callback
            RobotContainer.dt::tankDriveVolts,
            RobotContainer.dt);

    // Reset odometry to the starting pose of the trajectory.
    RobotContainer.dt.resetOdometry(newTrajectory.getInitialPose());

    addCommands(ramseteCommand.andThen(() -> RobotContainer.dt.tankDriveVolts(0, 0)));
  }
}
