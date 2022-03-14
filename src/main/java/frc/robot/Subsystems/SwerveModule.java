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

    private final RelativeEncoder driveEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);



        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new TalonSRX(turningMotorId);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
       

        driveEncoder.setPositionConversionFactor(ModuleConstants.const_DriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.const_DriveEncoderRPM2MeterPerSec);

        turningPidController = new PIDController(ModuleConstants.const_PTurning, ModuleConstants.const_ITurning, ModuleConstants.const_DTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        angle *= (absoluteEncoderReversed ? -1.0 : 1.0);
        if(angle<0){
            angle = 2*Math.PI - (-1*angle);
        }
        else if(angle > 2*Math.PI) {

            angle = angle - 2*Math.PI;
        }

        angle -= Math.PI;
        DecimalFormat df = new DecimalFormat("#.#");
        angle = Double.valueOf(df.format(angle));
        return angle;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

   /** public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity();
    }
 
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
*/
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            SmartDashboard.putString("Swerve" + absoluteEncoder.getChannel() + " state", state.toString());
            SmartDashboard.putNumber("Swerve" + absoluteEncoder.getChannel() + " angle", getTurningPosition());
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.const_PhysicalMaxSpeedMetersPerSecond);

        //turningMotor.set(ControlMode.PercentOutput);
        double test = MathUtil.clamp(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()), -1, 1);
        SmartDashboard.putNumber("test" + absoluteEncoder.getChannel(), test);

        turningMotor.set(ControlMode.PercentOutput, test);
        SmartDashboard.putString("Swerve" + absoluteEncoder.getChannel() + " state", state.toString());
        SmartDashboard.putNumber("Swerve" + absoluteEncoder.getChannel() + " angle", getTurningPosition());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
