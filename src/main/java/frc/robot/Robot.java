// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.BallHandler;
import frc.robot.Subsystems.SwerveSystem;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();



  private SwerveSystem swerveSubsystem = new SwerveSystem();
  private BallHandler BallSubsystem = new BallHandler();
  private Joystick driverJoytick = new Joystick(OIConstants.const_DriverControllerPort);

  //private final TalonSRX testMotor = new TalonSRX(8);
 

  private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.const_TeleDriveMaxAccelerationUnitsPerSecond);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.const_TeleDriveMaxAccelerationUnitsPerSecond);
  private SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.const_TeleDriveMaxAngularAccelerationUnitsPerSecond);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   
     // 1. Get real-time joystick inputs
     double xSpeed = driverJoytick.getRawAxis(OIConstants.const_DriverXAxis);
     //testMotor.set(ControlMode.PercentOutput, xSpeed);

     double ySpeed = driverJoytick.getRawAxis(OIConstants.const_DriverYAxis);
     double turningSpeed = driverJoytick.getRawAxis(OIConstants.const_DriverRotAxis);

     // 2. Apply deadband
     xSpeed = Math.abs(xSpeed) > OIConstants.const_Deadband ? xSpeed : 0.0;
     ySpeed = Math.abs(ySpeed) > OIConstants.const_Deadband ? ySpeed : 0.0;
     turningSpeed = Math.abs(turningSpeed) > OIConstants.const_Deadband ? turningSpeed : 0.0;

     // 3. Make the driving smoother
     xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
     ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.const_TeleDriveMaxSpeedMetersPerSecond;
     turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.const_TeleDriveMaxAngularSpeedRadiansPerSecond;

     // 4. Construct desired chassis speeds
     ChassisSpeeds chassisSpeeds;

     // Relative to field
     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

     // Relative to robot
     //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

     // 5. Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = DriveConstants.const_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

     // 6. Output each module states to wheels
     swerveSubsystem.setModuleStates(moduleStates);
     
     swerveSubsystem.periodic();


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
