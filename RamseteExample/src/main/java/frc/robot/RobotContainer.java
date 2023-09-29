// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static DriveTrain dt = new DriveTrain();
  
  // Simulation could be a gamechanger!
  public DriveForward forward = new DriveForward(dt);
  public TurnCCW ccw = new TurnCCW(90, dt, false);

  public Ramsete ramsete = new Ramsete();
  public TankDrive tankDrive = new TankDrive(dt);

  public static Joystick joy1 = new Joystick(0);
  public static Joystick joy2 = new Joystick(1);

  public RobotContainer() {
    dt.setDefaultCommand(tankDrive);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return ramsete;
  }

  public static Joystick getJoy1() {
    return joy1;
  }

    public static Joystick getJoy2() {
    return joy2;
  }

}
