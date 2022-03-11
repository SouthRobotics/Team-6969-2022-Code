package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class BallHandler {

    private final TalonSRX TopMotor1;
    private final TalonSRX BottomMotor1;
    private final TalonSRX TopMotor2;
    private final TalonSRX BottomMotor2;
    private final TalonSRX IntakeMotor;


    public BallHandler(int TopMotor1Id, int BottomMotor1Id, int TopMotor2Id, int BottomMotor2Id
    , boolean TopMotor1Reversed, boolean BottomMotor1Reversed, boolean TopMotor2Reversed, boolean BottomMotor2Reversed
    ,int IntakeMotorMotorId, boolean IntakeMotorMotorReversed
    ) {

        TopMotor1 = new TalonSRX(TopMotor1Id);
        BottomMotor1 = new TalonSRX(BottomMotor1Id);
        TopMotor2 = new TalonSRX(TopMotor2Id);
        BottomMotor2 = new TalonSRX(BottomMotor2Id);
        IntakeMotor = new TalonSRX(IntakeMotorMotorId);

        IntakeMotor.setInverted(IntakeMotorMotorReversed);
        TopMotor1.setInverted(TopMotor1Reversed);
        BottomMotor1.setInverted(BottomMotor1Reversed);
        TopMotor2.setInverted(TopMotor2Reversed);
        BottomMotor1.setInverted(BottomMotor2Reversed);

    }


    
    public void intake() {

        //TopMotor.set(ControlMode.PercentOutput, .5);
        //BottomMotor.set(ControlMode.PercentOutput, .5);

    }

    public void stop() {

        //TopMotor.set(ControlMode.PercentOutput, 0);
        //BottomMotor.set(ControlMode.PercentOutput, 0);

    }
    
}
