package frc.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.*;

public class Climber {

    private final TalonSRX Climber;
    private int counter;
    private int limit;
    
    public Climber() {

        Climber = new TalonSRX(ClimberConstants.const_ClimberId);

        
    }

    public void Climb(double power){

            Climber.set(ControlMode.PercentOutput, power);
        }

    
    
}
