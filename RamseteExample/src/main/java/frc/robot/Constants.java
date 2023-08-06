// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveTrainPorts {
        public static final int LeftDriveTalonPort = 1;
        public static final int RightDriveTalonPort = 2;
        public static final int LeftDriveVictorPort = 4;
        public static final int RightDriveVictorPort = 5;
    }

    public static final class DriveToLineConstants {
        public static final double wheelDiameterInInches = 6.0;
        public static final double ticksToMeters = (1.0 / (wheelDiameterInInches * 0.0254 * Math.PI)) * 4096.0;
    }

    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
    }

    public static final class RamseteConstants {
        // PLACEHOLDER VALUES FOR SIMULATION
        public static final double kS = 0.22;
        public static final double kV = 2.98;
        public static final double kA = 0.2;

        public static final double kVangular = 1.0;
        public static final double kAangular = 1.0;

        public static final double kPVel = 4.6;

        public static final double kMaxSpeed = 2; // In meters per second
        public static final double kMaxAcceleration = 3; // 3 In meters per second per second

        public static final double kTrackwidthMeters = 0.69; // Average chassis width - From AndyMark measurements
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

    }
}
