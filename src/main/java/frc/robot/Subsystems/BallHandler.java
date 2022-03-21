package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.BallHandlerConstants;
import com.ctre.phoenix.motorcontrol.*;
public class BallHandler {

    private final TalonSRX TopMotor1;
    private final TalonSRX BottomMotor1;
    private final TalonSRX TopMotor2;
    private final TalonSRX BottomMotor2;
    private final TalonSRX IntakeAngleMotor;
    private final TalonSRX IntakeDriveMotor;
    private final TalonSRX HoodAngleMotor;
    private final TalonSRX HoodDriveMotor;
    private final boolean IntakeDeployed;
    private final DigitalInput Ball1Loaded;
    private final DigitalInput Ball2Loaded;

    public BallHandler() {

        TopMotor1 = new TalonSRX(TopMotor1Id);
        BottomMotor1 = new TalonSRX(BottomMotor1Id);
        TopMotor2 = new TalonSRX(TopMotor2Id);
        BottomMotor2 = new TalonSRX(BottomMotor2Id);
        
        IntakeAngleMotor = new TalonSRX(IntakeAngleMotorId);
        IntakeDriveMotor = new TalonSRX(IntakeDriveMotorId);

        HoodAngleMotor = new TalonSRX(HoodAngleMotorId); 
        HoodDriveMotor = new TalonSRX(HoodDriveMotorId); 

        IntakeAngleMotor.setInverted(IntakeAngleMotorReversed);
        IntakeDriveMotor.setInverted(IntakeDriveMotorReversed);
        HoodAngleMotor.setInverted(HoodAngleMotorReversed);
        HoodDriveMotor.setInverted(HoodDriveMotorReversed);
        TopMotor1.setInverted(TopMotor1Reversed);
        BottomMotor1.setInverted(BottomMotor1Reversed);
        TopMotor2.setInverted(TopMotor2Reversed);
        BottomMotor1.setInverted(BottomMotor2Reversed);


        Ball1Loaded = new DigitalInput(BallHandlerConstants.const_Ball1Input);
        Ball2Loaded = new DigitalInput(BallHandlerConstants.const_Ball2Input);

        //set up PID for hood and intake angle and launcher motor

        resetEncoders();
        IntakeDeployed = false;


    }


    
    public void intake(boolean state) {

        if (state) {
            moveConveyer(false);
            if(!Ball1Loaded.get()) {
                IntakeDriveMotor.set(ControlMode.PercentOutput, BallHandlerConstants.const_IntakeSpeedPercent);
            }
        }
        else{

            TopMotor1.set(ControlMode.PercentOutput, 0);
            BottomMotor1.set(ControlMode.PercentOutput,0);
            TopMotor2.set(ControlMode.PercentOutput, 0);
            BottomMotor2.set(ControlMode.PercentOutput,0);
            IntakeDriveMotor.set(ControlMode.PercentOutput, 0);
        }
    }


    public void launch(double launchVel) {

        double launchSpeed = ((launchVel * 60)/(2*Math.PI*BallHandlerConstants.const_LauncherWheelDiamIN))* 4096 / 600;

        double error = launchSpeed - HoodDriveMotor.getSelectedSensorVelocity();
        
        if ( error < 10 ){
            moveConveyer(true);
        }
        else{
            moveConveyer(false);
        }
        HoodDriveMotor.set(ControlMode.Velocity, launchSpeed);

    }
    

    public void moveConveyer(boolean launchMove) {

        if (launchMove){

            TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            TopMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            BottomMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
        }
        else {

            if(!Ball2Loaded.get()) {
                TopMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                BottomMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            }
            else{
                TopMotor2.set(ControlMode.PercentOutput, 0);
                BottomMotor2.set(ControlMode.PercentOutput,0);
            }



            if(!Ball1Loaded.get()) {
                TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            }
            else{
                
                if(!Ball2Loaded.get()) {
                    TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                    BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                }
                else{
                    TopMotor1.set(ControlMode.PercentOutput, 0);
                    BottomMotor1.set(ControlMode.PercentOutput,0);
                }
            }



        }

    }

    
    public void setHoodAngle(double angle) {

        if (angle > BallHandlerConstants.const_HoodAngleMax){
            angle = BallHandlerConstants.const_HoodAngleMax;
        }

        HoodAngleMotor.set(ControlMode.Position, (angle/360)* 4096);

    }

    public void deployIntake() {
        if (!IntakeDeployed){
            IntakeAngleMotor.set(ControlMode.Position, (BallHandlerConstants.const_IntakeDeployAngle/360)* 4096);
        }
    }
    public void stowIntake() {
        if (IntakeDeployed){
            IntakeAngleMotor.set(ControlMode.Position, (BallHandlerConstants.const_IntakeStowAngle/360)* 4096);
        }
    }

    public void resetEncoders() {
        HoodAngleMotor.setSelectedSensorPosition((BallHandlerConstants.const_HoodAngleOffset/360)* 4096);
        IntakeAngleMotor.setSelectedSensorPosition(0);
    }


    public void stop() {

        //TopMotor.set(ControlMode.PercentOutput, 0);
        //BottomMotor.set(ControlMode.PercentOutput, 0);

    }




    
}
