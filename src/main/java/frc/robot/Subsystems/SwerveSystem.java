package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.const_FrontLeftDriveMotorPort,
            DriveConstants.const_FrontLeftTurningMotorPort,
            DriveConstants.const_FrontLeftDriveEncoderReversed,
            DriveConstants.const_FrontLeftTurningEncoderReversed,
            DriveConstants.const_FrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.const_FrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.const_FrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.const_FrontRightDriveMotorPort,
            DriveConstants.const_FrontRightTurningMotorPort,
            DriveConstants.const_FrontRightDriveEncoderReversed,
            DriveConstants.const_FrontRightTurningEncoderReversed,
            DriveConstants.const_FrontRightDriveAbsoluteEncoderPort,
            DriveConstants.const_FrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.const_FrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.const_BackLeftDriveMotorPort,
            DriveConstants.const_BackLeftTurningMotorPort,
            DriveConstants.const_BackLeftDriveEncoderReversed,
            DriveConstants.const_BackLeftTurningEncoderReversed,
            DriveConstants.const_BackLeftDriveAbsoluteEncoderPort,
            DriveConstants.const_BackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.const_BackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.const_BackRightDriveMotorPort,
            DriveConstants.const_BackRightTurningMotorPort,
            DriveConstants.const_BackRightDriveEncoderReversed,
            DriveConstants.const_BackRightTurningEncoderReversed,
            DriveConstants.const_BackRightDriveAbsoluteEncoderPort,
            DriveConstants.const_BackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.const_BackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.const_DriveKinematics, new Rotation2d(0));

    public SwerveSystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.const_PhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
