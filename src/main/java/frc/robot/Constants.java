package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double const_WheelDiameterMeters = Units.inchesToMeters(4);
        public static final double const_DriveMotorGearRatio = 1 / 6.67;
        public static final double const_TurningMotorGearRatio = 1 / 1.2;
        public static final double const_DriveEncoderRot2Meter = const_DriveMotorGearRatio * Math.PI * const_WheelDiameterMeters;
        public static final double const_TurningEncoderRot2Rad = const_TurningMotorGearRatio * 2 * Math.PI;
        public static final double const_DriveEncoderRPM2MeterPerSec = const_DriveEncoderRot2Meter / 60;
        public static final double const_TurningEncoderRPM2RadPerSec = const_TurningEncoderRot2Rad / 60;
        public static final double const_PTurning = 0.5;
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double const_TrackWidth = Units.inchesToMeters(19);
        // Distance between front and back wheels
        public static final double const_WheelBase = Units.inchesToMeters(29.11);
        
        public static final SwerveDriveKinematics const_DriveKinematics = new SwerveDriveKinematics(
                new Translation2d(const_WheelBase / 2, -const_TrackWidth / 2),
                new Translation2d(const_WheelBase / 2, const_TrackWidth / 2),
                new Translation2d(-const_WheelBase / 2, -const_TrackWidth / 2),
                new Translation2d(-const_WheelBase / 2, const_TrackWidth / 2));

        public static final int const_FrontLeftDriveMotorPort = 8;
        public static final int const_BackLeftDriveMotorPort = 2;
        public static final int const_FrontRightDriveMotorPort = 6;
        public static final int const_BackRightDriveMotorPort = 4;

        public static final int const_FrontLeftTurningMotorPort = 7;
        public static final int const_BackLeftTurningMotorPort = 1;
        public static final int const_FrontRightTurningMotorPort = 5;
        public static final int const_BackRightTurningMotorPort = 3;

        public static final boolean const_FrontLeftTurningEncoderReversed = true;
        public static final boolean const_BackLeftTurningEncoderReversed = true;
        public static final boolean const_FrontRightTurningEncoderReversed = true;
        public static final boolean const_BackRightTurningEncoderReversed = true;

        public static final boolean const_FrontLeftDriveEncoderReversed = true;
        public static final boolean const_BackLeftDriveEncoderReversed = true;
        public static final boolean const_FrontRightDriveEncoderReversed = false;
        public static final boolean const_BackRightDriveEncoderReversed = false;

        public static final int const_FrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int const_BackLeftDriveAbsoluteEncoderPort = 2;
        public static final int const_FrontRightDriveAbsoluteEncoderPort = 1;
        public static final int const_BackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean const_FrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean const_BackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean const_FrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean const_BackRightDriveAbsoluteEncoderReversed = false;

        public static final double const_FrontLeftDriveAbsoluteEncoderOffsetRad = 1; //need to set
        public static final double const_BackLeftDriveAbsoluteEncoderOffsetRad = 1; //need to set
        public static final double const_FrontRightDriveAbsoluteEncoderOffsetRad = 1; //need to set
        public static final double const_BackRightDriveAbsoluteEncoderOffsetRad = 1; //need to set

        public static final double const_PhysicalMaxSpeedMetersPerSecond = 8;
        public static final double const_PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double const_TeleDriveMaxSpeedMetersPerSecond = const_PhysicalMaxSpeedMetersPerSecond / 4;
        public static final double const_TeleDriveMaxAngularSpeedRadiansPerSecond = const_PhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double const_TeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double const_TeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double const_MaxSpeedMetersPerSecond = DriveConstants.const_PhysicalMaxSpeedMetersPerSecond / 4;
        public static final double const_MaxAngularSpeedRadiansPerSecond = DriveConstants.const_PhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double const_MaxAccelerationMetersPerSecondSquared = 3;
        public static final double const_MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double const_PXController = 1.5;
        public static final double const_PYController = 1.5;
        public static final double const_PThetaController = 3;

        public static final TrapezoidProfile.Constraints const_ThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        const_MaxAngularSpeedRadiansPerSecond,
                        const_MaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int const_DriverControllerPort = 0;

        public static final int const_DriverYAxis = 1;
        public static final int const_DriverXAxis = 0;
        public static final int const_DriverRotAxis = 4;
        public static final int const_DriverFieldOrientedButtonIdx = 1;

        public static final double const_Deadband = 0.05;
    }

}
