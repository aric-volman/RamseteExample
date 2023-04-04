// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final WPI_VictorSPX leftVictor;
  private final WPI_VictorSPX rightVictor;

  private final DifferentialDrive drive;

  private final DifferentialDriveOdometry odometry;

  private static AHRS navx = new AHRS(SPI.Port.kMXP);

  public DriveTrain() {

    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    leftVictor = new WPI_VictorSPX(Constants.DriveTrainPorts.LeftDriveVictorPort);
    rightVictor = new WPI_VictorSPX(Constants.DriveTrainPorts.RightDriveVictorPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftVictor.follow(leftDriveTalon);
    rightVictor.follow(rightDriveTalon);
    
    drive = new DifferentialDrive(leftDriveTalon, rightDriveTalon);

    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());

  }

  public static double getAngle() {
    return navx.getAngle();
  }

  public double getLeftDistance() {
    return leftDriveTalon.getSelectedSensorPosition()/Constants.DriveToLineConstants.ticksToMeters;
  }

  public double getRightDistance() {
    return rightDriveTalon.getSelectedSensorPosition()/Constants.DriveToLineConstants.ticksToMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
  }
}
