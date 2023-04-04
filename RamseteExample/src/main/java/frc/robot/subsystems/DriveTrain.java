// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final WPI_VictorSPX leftVictor;
  private final WPI_VictorSPX rightVictor;

  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;
  private final DifferentialDrivetrainSim driveSim;

  private final DifferentialDrive drive;

  private final DifferentialDriveOdometry odometry;

  private static AHRS navx = new AHRS(SPI.Port.kMXP);

  private Field2d field = new Field2d();

  private double simRightVolts;
  private double simLeftVolts;

  public DriveTrain() {

    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    leftVictor = new WPI_VictorSPX(Constants.DriveTrainPorts.LeftDriveVictorPort);
    rightVictor = new WPI_VictorSPX(Constants.DriveTrainPorts.RightDriveVictorPort);

      leftDriveSim = leftDriveTalon.getSimCollection();
      rightDriveSim = rightDriveTalon.getSimCollection();
      driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),        // 2 CIMS on each side of the drivetrain.
        10.71,               //Standard AndyMark Gearing reduction.
        2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
        26.5,                     //Mass of the robot is 26.5 kg.
        Units.inchesToMeters(6.0),  //Robot uses 3" radius (6" diameter) wheels.
        0.69, null
      );
  
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

    resetEncoders();
    
    drive = new DifferentialDrive(leftDriveTalon, rightDriveTalon);

    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());

  }

  /**
   * Resets the chassis encoders to 0 ticks.
   */
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0);
    rightDriveTalon.setSelectedSensorPosition(0);
  }

  /**
   * Returns angular displacement of chassis.
   * 
   * @return the angular displacement (degrees)
   */
  public static double getAngle() {
    return navx.getAngle();
  }

  /**
   * Returns displacement of left side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getLeftDistance() {
    return leftDriveTalon.getSelectedSensorPosition()/Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns displacement of right side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getRightDistance() {
    return rightDriveTalon.getSelectedSensorPosition()/Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns linear velocity of left side of chassis.
   * 
   * @return the linear velocity in meters/s (m/s)
   */
  public double getLeftSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity()*10.0)/Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns linear velocity of right side of chassis.
   * 
   * @return the linear velocity in meters/s (m/s)
   */
  public double getRightSpeed() {
    return (rightDriveTalon.getSelectedSensorVelocity()*10.0)/Constants.DriveToLineConstants.ticksToMeters;
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
    // This method will be called once per scheduler run

    odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
    SmartDashboard.putData("Field", field);
    field.setRobotPose(getPose());
    }
  }

  @Override
  public void simulationPeriodic() {
    // Sim Stuff
    // https://www.chiefdelphi.com/t/drivebase-simulation-example-with-talonsrx-encoders/390390/10

    driveSim.setInputs(simLeftVolts, simRightVolts);
    driveSim.update(0.020);

    // Update Quadrature
    leftDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(
          driveSim.getLeftPositionMeters()
      ));
leftDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          driveSim.getLeftVelocityMetersPerSecond()
      ));
rightDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(
          -driveSim.getRightPositionMeters()
      ));
rightDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          -driveSim.getRightVelocityMetersPerSecond()
      ));
      driveSim.update(0.02);

      // Update Gyro
      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
      angle.set(-driveSim.getHeading().getDegrees());

      odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
      SmartDashboard.putData("Field", field);
      field.setRobotPose(driveSim.getPose());
      SmartDashboard.putNumber("Heading:", driveSim.getHeading().getDegrees());
  }

  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        navx.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * Voltage is the native unit of Feedforward with WPILib
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //drive.setSafetyEnabled(false);
    simLeftVolts = leftVolts;
    simRightVolts = rightVolts;
    leftDriveTalon.setVoltage(leftVolts);
    rightDriveTalon.setVoltage(rightVolts);
    // WPILib would spit out "Looptime Overrun!" if this isn't included!
    drive.feed();
  }

  // CTRE SIM methods:

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(6.0));
    double motorRotations = wheelRotations * 1.0;
    int sensorCounts = (int)(motorRotations * 4096.0);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(6.0));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 4096.0;
    double wheelRotations = motorRotations / 1.0;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(6.0));
    return positionMeters;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate();
  }
}
