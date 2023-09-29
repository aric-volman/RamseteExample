// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnCCW extends CommandBase {
  DriveTrain dt;
  PIDController pid = new PIDController(0.2, 0.0, 0.04);

  double angle = 0.0;

  boolean reset = true;

  double motorSign = 1.0;

  /** Creates a new TurnCCW. */
  public TurnCCW(double angle, DriveTrain dt, boolean reset) {
    this.dt = dt;
    this.angle = angle;
    this.reset = reset;

    if (angle > 0) {
      motorSign = 1.0;
    } else if (angle <= 0) {
      motorSign = -1.0;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (reset) {dt.zeroHeading();}
    dt.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = pid.calculate(dt.getHeading(), angle);

    dt.tankDriveVolts(-output*motorSign, output*motorSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
