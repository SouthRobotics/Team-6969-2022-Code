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
            DriveConstants.Swerve_FL.const_DriveMotorPort,
            DriveConstants.Swerve_FL.const_TurningMotorPort,
            DriveConstants.Swerve_FL.const_DriveMotorReversed,
            DriveConstants.Swerve_FL.const_TurningMotorReversed,
            DriveConstants.Swerve_FL.const_AbsoluteEncoderPort,
            DriveConstants.Swerve_FL.const_AbsoluteEncoderOffsetRad,
            DriveConstants.Swerve_FL.const_AbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.Swerve_FR.const_DriveMotorPort,
            DriveConstants.Swerve_FR.const_TurningMotorPort,
            DriveConstants.Swerve_FR.const_DriveMotorReversed,
            DriveConstants.Swerve_FR.const_TurningMotorReversed,
            DriveConstants.Swerve_FR.const_AbsoluteEncoderPort,
            DriveConstants.Swerve_FR.const_AbsoluteEncoderOffsetRad,
            DriveConstants.Swerve_FR.const_AbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.Swerve_BL.const_DriveMotorPort,
            DriveConstants.Swerve_BL.const_TurningMotorPort,
            DriveConstants.Swerve_BL.const_DriveMotorReversed,
            DriveConstants.Swerve_BL.const_TurningMotorReversed,
            DriveConstants.Swerve_BL.const_AbsoluteEncoderPort,
            DriveConstants.Swerve_BL.const_AbsoluteEncoderOffsetRad,
            DriveConstants.Swerve_BL.const_AbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.Swerve_BR.const_DriveMotorPort,
            DriveConstants.Swerve_BR.const_TurningMotorPort,
            DriveConstants.Swerve_BR.const_DriveMotorReversed,
            DriveConstants.Swerve_BR.const_TurningMotorReversed,
            DriveConstants.Swerve_BR.const_AbsoluteEncoderPort,
            DriveConstants.Swerve_BR.const_AbsoluteEncoderOffsetRad,
            DriveConstants.Swerve_BR.const_AbsoluteEncoderReversed);

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
