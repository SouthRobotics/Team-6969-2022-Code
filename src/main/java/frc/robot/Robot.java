// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.BallHandler;
import frc.robot.Subsystems.SwerveSystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Timer timer = new Timer();


  private SwerveSystem swerveSubsystem = new SwerveSystem();
  private BallHandler BallSubsystem = new BallHandler();
  private Joystick driverJoytick = new Joystick(OIConstants.const_DriverControllerPort);
  private Joystick yawstick = new Joystick(1);

  private TalonSRX Climber = new TalonSRX(Constants.ClimberConstants.const_ClimberId);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(1, 0.0, 0,
      new TrapezoidProfile.Constraints(DriveConstants.const_PhysicalMaxAngularSpeedRadiansPerSecond,
          DriveConstants.const_TeleDriveMaxAngularAccelerationUnitsPerSecond));
  //private final CANSparkMax testMotor = new CANSparkMax(3, MotorType.kBrushless);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.const_TeleDriveMaxAccelerationUnitsPerSecond);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.const_TeleDriveMaxAccelerationUnitsPerSecond);
  private SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.const_TeleDriveMaxAngularAccelerationUnitsPerSecond);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("Pvalue", .35);
    SmartDashboard.putNumber("Ivalue", 0);
    SmartDashboard.putNumber("Dvalue", .05);

    SmartDashboard.putNumber("MotorSpeed", .75);

    SmartDashboard.putNumber("frontLeftOffset", 1.641539);
    SmartDashboard.putNumber("frontRightOffset", -2.279243);
    SmartDashboard.putNumber("backLeftOffset", -.982306);
    SmartDashboard.putNumber("backRightOffset", 1.45);

    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setFPS(10);
    camera1.setResolution(160, 120);
    camera2 = CameraServer.startAutomaticCapture(1);
    camera2.setFPS(10);
    camera2.setResolution(160, 120);
    server = CameraServer.getServer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getRotation2d().getDegrees());
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(2, 0, 0);
    SwerveModuleState[] moduleStates = DriveConstants.const_DriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    swerveSubsystem.periodic();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /**
     * switch (m_autoSelected) {
     * case kCustomAuto:
     * // Put custom auto code here
     * break;
     * case kDefaultAuto:
     * default:
     * // Put default auto code here
     * break;
     * }
     */

     if(timer.get() > 1.5){
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
      SwerveModuleState[] moduleStates = DriveConstants.const_DriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerveSubsystem.setModuleStates(moduleStates);
      timer.stop();
     }
     else{
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2, 0, 0);
        SwerveModuleState[] moduleStates = DriveConstants.const_DriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
        swerveSubsystem.periodic();
     }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    BallSubsystem.periodictest();

    // 1. Get real-time joystick inputs
    double xSpeed = driverJoytick.getRawAxis(OIConstants.const_DriverYAxis)* DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
    // testMotor.set(ControlMode.PercentOutput, xSpeed);

    double ySpeed = driverJoytick.getRawAxis(OIConstants.const_DriverXAxis)* DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
    double turningSpeed = yawstick.getRawAxis(0)* DriveConstants.const_TeleDriveMaxAngularSpeedRadiansPerSecond;

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.const_Deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.const_Deadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.const_Deadband ? turningSpeed : 0.0;

    //testMotor.set(ySpeed);
    // 3. Make the driving smoother
     xSpeed = xLimiter.calculate(xSpeed) *DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
     ySpeed = yLimiter.calculate(ySpeed) *DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
     turningSpeed = turningLimiter.calculate(turningSpeed)*DriveConstants.const_TeleDriveMaxAngularSpeedRadiansPerSecond;

    if (driverJoytick.getRawButton(1)) { // shoot mode

      // System.out.println("hi");
      // Climber.set(ControlMode.PercentOutput, -.5);
      BallSubsystem.launch(SmartDashboard.getNumber("MotorSpeed", 0), false);

      // if target not detectd roate constantly, else pid loop for turning
      // double turn_output = m_controller.calculate(horizantal_angle);

      // if(aim is good){
      // BallSubsystem.launch(5, true);
      // }

    }

    else { 
      BallSubsystem.launch(0, false);
    }






    if (driverJoytick.getRawButton(3)) {
      BallSubsystem.intake(true, true);
      

    }
    else if (driverJoytick.getRawButton(4)) {
      BallSubsystem.intake(true, false);
      //System.out.println("hi");
    }
    else {
      BallSubsystem.intake(false, false);
    }
    // climber





    if (driverJoytick.getRawButton(2)) {
      BallSubsystem.deployIntake(true);

    } else {
      BallSubsystem.deployIntake(false);

    }


    if (driverJoytick.getRawButton(5)) {
      swerveSubsystem.zeroHeading();

    }


    //BallSubsystem.setHoodAngle(SmartDashboard.getNumber("Hood Angle", 0));
    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;

    // Relative to field
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

    // Relative to robot
    //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    //chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.const_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // SwerveModuleState[] moduleStates = {new SwerveModuleState(0, new
    // Rotation2d(xSpeed*3)),
    // new SwerveModuleState(0, new Rotation2d(xSpeed*3)),
    // new SwerveModuleState(0, new Rotation2d(xSpeed*3)),
    // new SwerveModuleState(0, new Rotation2d(xSpeed*3))};
    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

    swerveSubsystem.periodic();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
