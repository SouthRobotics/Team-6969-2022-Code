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
        public static final double const_PTurning = .01;
        public static final double const_ITurning = 0;
        public static final double const_DTurning = .05;
    }

    public static final class ClimberConstants {
        public static final int const_ClimberId = 14;
    }

    public static final class BallHandlerConstants {
        public static final double const_HoodAngleOffset= 0; //set
        public static final double const_HoodAngleMax= 90; //set
        public static final double const_HoodAngleRatio= 4.125;

        public static final double const_IntakeStowAngle= 0;
        public static final double const_IntakeDeployAngle = 90; //set
        public static final double const_LauncherWheelDiamIN = 4;
        public static final int const_Ball1Input = 0; //set
        public static final int const_Ball2Input = 1; //set
        public static final double const_ConveyerSpeedPercent = .75; //set
        public static final double const_IntakeSpeedPercent = .5; //set

        public static final int const_TopMotor1Id = 9; 
        public static final int const_BottomMotor1Id  = 11;
        public static final int const_TopMotor2Id  = 7; 
        public static final int const_BottomMotor2Id  = 8;
        
        public static final int const_IntakeAngleMotorId  = 15; 
        public static final int const_IntakeDriveMotorId  = 6; 

        public static final int const_HoodAngleMotorId  = 17; 
        public static final int const_HoodDriveMotorId  = 10; 

        public static final boolean const_IntakeAngleMotorReversed = false; 
        public static final boolean const_IntakeDriveMotorReversed = true; 
        public static final boolean const_HoodAngleMotorReversed = true; 
        public static final boolean const_HoodDriveMotorReversed = true; 
        public static final boolean const_TopMotor1Reversed = true; 
        public static final boolean const_BottomMotor1Reversed = true; 
        public static final boolean const_TopMotor2Reversed = true; 
        public static final boolean const_BottomMotor2Reversed = true; 
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double const_TrackWidth = Units.inchesToMeters(19);
        // Distance between front and back wheels
        public static final double const_WheelBase = Units.inchesToMeters(29.11);
        
        public static final SwerveDriveKinematics const_DriveKinematics = new SwerveDriveKinematics(
                //FrontLeft
                new Translation2d(const_WheelBase / 2, const_TrackWidth / 2),
                //FrontRight
                new Translation2d(const_WheelBase / 2, -const_TrackWidth / 2),
                //BackLeft
                new Translation2d(-const_WheelBase / 2, const_TrackWidth / 2),
                //BackRight
                new Translation2d(-const_WheelBase / 2, -const_TrackWidth / 2));


        
        public static final String[] swerveID =  {"FL", "FR", "BL", "BR", };

        public static final class Swerve_FL {
            public static final int const_DriveMotorPort = 2;
            public static final int const_TurningMotorPort = 13;
            public static final boolean const_TurningMotorReversed = true;
            public static final boolean const_DriveMotorReversed = false;
            public static final int const_AbsoluteEncoderPort = 0;
            public static final boolean const_AbsoluteEncoderReversed = false;
            public static final double const_AbsoluteEncoderOffsetRad = 1.49-.325;
        }
        public static final class Swerve_FR {
            public static final int const_DriveMotorPort = 1;
            public static final int const_TurningMotorPort = 12;
            public static final boolean const_TurningMotorReversed = true;
            public static final boolean const_DriveMotorReversed = true;
            public static final int const_AbsoluteEncoderPort = 3;
            public static final boolean const_AbsoluteEncoderReversed = false;
            public static final double const_AbsoluteEncoderOffsetRad = 1.744-3.01;
        }
        public static final class Swerve_BL {
            public static final int const_DriveMotorPort = 3;
            public static final int const_TurningMotorPort = 16;
            public static final boolean const_TurningMotorReversed = true;
            public static final boolean const_DriveMotorReversed = false;
            public static final int const_AbsoluteEncoderPort = 1;
            public static final boolean const_AbsoluteEncoderReversed = false;
            public static final double const_AbsoluteEncoderOffsetRad = 4.43-1.407;
        }
        public static final class Swerve_BR {
            public static final int const_DriveMotorPort = 4;
            public static final int const_TurningMotorPort = 5;
            public static final boolean const_TurningMotorReversed = true;
            public static final boolean const_DriveMotorReversed = true;
            public static final int const_AbsoluteEncoderPort = 2;
            public static final boolean const_AbsoluteEncoderReversed = false;
            public static final double const_AbsoluteEncoderOffsetRad = .678+1.730;
        }


        public static final double const_PhysicalMaxSpeedMetersPerSecond = 4;
        public static final double const_PhysicalMaxAngularSpeedRadiansPerSecond = Math.PI;

        public static final double const_TeleDriveMaxSpeedMetersPerSecond = const_PhysicalMaxSpeedMetersPerSecond /2;
        public static final double const_TeleDriveMaxAngularSpeedRadiansPerSecond = const_PhysicalMaxAngularSpeedRadiansPerSecond /2;
        
        public static final double const_TeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double const_TeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    /**public static final class AutoConstants {
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
    */

    public static final class OIConstants {
        public static final int const_DriverControllerPort = 0;

        public static final int const_DriverYAxis = 1;
        public static final int const_DriverXAxis = 0;
        public static final int const_DriverRotAxis = 2;
        

        public static final double const_Deadband = 0.1;
    }

}
