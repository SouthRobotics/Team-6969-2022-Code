package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.MathUtil;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.*;


public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final TalonSRX turningMotor;
    private final String name;
    private final RelativeEncoder driveEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        this.name = name;


        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new TalonSRX(turningMotorId);
        turningMotor.configFactoryDefault();


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
       

        driveEncoder.setPositionConversionFactor(ModuleConstants.const_DriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.const_DriveEncoderRPM2MeterPerSec);

        
        //turningPidController = new PIDController(ModuleConstants.const_PTurning, ModuleConstants.const_ITurning, ModuleConstants.const_DTurning);
        turningPidController = new PIDController(SmartDashboard.getNumber("Pvalue", 0), SmartDashboard.getNumber("Ivalue", 0), SmartDashboard.getNumber("Dvalue", 0));
        
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return 0;
    }

    public double getTurningPosition() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        double offset = SmartDashboard.getNumber(name + "Offset", 0);
        //angle -= absoluteEncoderOffsetRad;
        angle -= offset;
        angle *= (absoluteEncoderReversed ? -1.0 : 1.0);
        if(angle<0){
            angle = 2*Math.PI - (-1*angle);
        }
        else if(angle > 2*Math.PI) {

            angle = angle - 2*Math.PI;
        }

        angle -= Math.PI;
        //DecimalFormat df = new DecimalFormat("#.#");
        //angle = Double.valueOf(df.format(angle));
        return angle;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //if (Math.abs(state.speedMetersPerSecond) < 0.001) {
         //   SmartDashboard.putString("Swerve " + name + " wanted state", state.toString());
          //  SmartDashboard.putNumber("Swerve " + name + " wanted angle", state.angle.getRadians());
           // SmartDashboard.putNumber("Swerve " + name + " current angle", getTurningPosition());
            //SmartDashboard.putNumber("Swerve " + name + " current speed", getDriveVelocity());
            //stop();
            //return;
        //}
        turningPidController.setI(SmartDashboard.getNumber("Ivalue", 0));
        turningPidController.setD(SmartDashboard.getNumber("Dvalue", 0));
        turningPidController.setP(SmartDashboard.getNumber("Pvalue", 0));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.const_PhysicalMaxSpeedMetersPerSecond);

        double turnmotorspeed = MathUtil.clamp(this.turningPidController.calculate(getTurningPosition(), state.angle.getRadians()), -1, 1);
        SmartDashboard.putNumber("test" + name, turnmotorspeed);

        turningMotor.set(ControlMode.PercentOutput, turnmotorspeed);
        SmartDashboard.putString("Swerve " + name + " wanted state", state.toString());
        SmartDashboard.putNumber("Swerve " + name + " wanted angle", state.angle.getDegrees());
        SmartDashboard.putNumber("Swerve " + name + " current angle", getTurningPosition() * (180/Math.PI));
        SmartDashboard.putNumber("Swerve " + name + " current speed", getDriveVelocity());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
