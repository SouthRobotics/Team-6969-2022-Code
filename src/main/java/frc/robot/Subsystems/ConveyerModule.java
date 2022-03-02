package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix.motorcontrol.*;

public class ConveyerModule {

    private final TalonSRX TopMotor;
    private final TalonSRX BottomMotor;


    public ConveyerModule(int TopMotorId, int BottomMotorId, boolean TopMotorReversed, boolean BottomMotorReversed) {

        TopMotor = new TalonSRX(TopMotorId);
        BottomMotor = new TalonSRX(BottomMotorId);


        TopMotor.setInverted(TopMotorReversed);
        BottomMotor.setInverted(BottomMotorReversed);

    }

    public void setState(double state) {

        TopMotor.set(ControlMode.PercentOutput, state);
        BottomMotor.set(ControlMode.PercentOutput, state);

    }
    
}
