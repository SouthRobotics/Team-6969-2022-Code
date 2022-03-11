package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
