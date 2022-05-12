package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.BallHandlerConstants;
import com.ctre.phoenix.motorcontrol.*;
public class BallHandler {

    private final TalonSRX TopMotor1;
    private final TalonSRX BottomMotor1;
    private final TalonSRX TopMotor2;
    private final TalonSRX BottomMotor2;
    private final TalonSRX IntakeAngleMotor;
    private final TalonSRX IntakeDriveMotor;
    private final TalonSRX HoodDriveMotor2;
    private final TalonSRX HoodDriveMotor;
    private int IntakeDeployed;
    private final DigitalInput Ball1Loaded;
    private final DigitalInput Ball2Loaded;

    BangBangController BangBang = new BangBangController();
    //PIDController hoodangle = new PIDController(SmartDashboard.getNumber("PvalueForHood", .0), 0, 0);

    public BallHandler() {

        TopMotor1 = new TalonSRX(BallHandlerConstants.const_TopMotor1Id);
        BottomMotor1 = new TalonSRX(BallHandlerConstants.const_BottomMotor1Id);
        TopMotor2 = new TalonSRX(BallHandlerConstants.const_TopMotor2Id);
        BottomMotor2 = new TalonSRX(BallHandlerConstants.const_BottomMotor2Id);
        
        IntakeAngleMotor = new TalonSRX(BallHandlerConstants.const_IntakeAngleMotorId);
        IntakeDriveMotor = new TalonSRX(BallHandlerConstants.const_IntakeDriveMotorId);
        IntakeDriveMotor.configFactoryDefault();

        HoodDriveMotor2 = new TalonSRX(BallHandlerConstants.const_HoodAngleMotorId); 
        HoodDriveMotor = new TalonSRX(BallHandlerConstants.const_HoodDriveMotorId); 

        IntakeAngleMotor.setInverted(BallHandlerConstants.const_IntakeAngleMotorReversed);
        IntakeDriveMotor.setInverted(BallHandlerConstants.const_IntakeDriveMotorReversed);
        HoodDriveMotor2.setInverted(BallHandlerConstants.const_HoodAngleMotorReversed);
        HoodDriveMotor.setInverted(BallHandlerConstants.const_HoodDriveMotorReversed);
        TopMotor1.setInverted(BallHandlerConstants.const_TopMotor1Reversed);
        BottomMotor1.setInverted(BallHandlerConstants.const_BottomMotor1Reversed);
        TopMotor2.setInverted(BallHandlerConstants.const_TopMotor2Reversed);
        BottomMotor2.setInverted(BallHandlerConstants.const_BottomMotor2Reversed);


        Ball1Loaded = new DigitalInput(BallHandlerConstants.const_Ball1Input);
        Ball2Loaded = new DigitalInput(BallHandlerConstants.const_Ball2Input);

        //set up PID for hood and intake angle and launcher motor

        //IntakeAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
        //HoodAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
        HoodDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
        //HoodAngleMotor.setSensorPhase(false); //set

        resetEncoders();
        IntakeDeployed = 0;


    }


    public void periodictest(){

        SmartDashboard.putBoolean("Ball 1 Loaded", Ball1Loaded.get());
        SmartDashboard.putBoolean("Ball 2 Loaded", Ball2Loaded.get());
        //hoodangle.setP(SmartDashboard.getNumber("PvalueForHood",0));
        //SmartDashboard.putNumber("Hood Current", HoodAngleMotor.getSelectedSensorPosition());
    }
    
    public void intake(boolean state, boolean type) {

        

        if (state) {
            moveConveyer(type);
           
            IntakeDriveMotor.set(ControlMode.PercentOutput, BallHandlerConstants.const_IntakeSpeedPercent);
            
        }
        else{

            TopMotor1.set(ControlMode.PercentOutput, 0);
            BottomMotor1.set(ControlMode.PercentOutput,0);
            TopMotor2.set(ControlMode.PercentOutput, 0);
            BottomMotor2.set(ControlMode.PercentOutput,0);
            IntakeDriveMotor.set(ControlMode.PercentOutput, 0);
        }
    }


    public void launch(double launchVel, boolean ontarget) {

        double launchSpeed = ((launchVel * 60)/(2*Math.PI*BallHandlerConstants.const_LauncherWheelDiamIN))* 4096 / 600;

        //double error = launchSpeed - HoodDriveMotor.getSelectedSensorVelocity();
        
        if (launchVel != 0){

            //if ( error < 10 && ontarget){
              //  moveConveyer(true);
                
            //}
            //else{
             //   moveConveyer(false);
            //}
            HoodDriveMotor.set(ControlMode.PercentOutput,  launchVel);
            

        }
        else{
            HoodDriveMotor.set(ControlMode.PercentOutput, 0);
        }


        
    }
    

    public void moveConveyer(boolean launchMove) {
        
        if (launchMove){
            

            TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            TopMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            BottomMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
        }
        else {
            TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            TopMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
           /*

            //System.out.println(Ball2Loaded.get());
            if(Ball2Loaded.get()) {
                TopMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                BottomMotor2.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            }
            else{
                TopMotor2.set(ControlMode.PercentOutput, 0);
                BottomMotor2.set(ControlMode.PercentOutput,0);
            }



            if(Ball1Loaded.get()) {
                TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
            }
            else{
                
                if(Ball2Loaded.get()) {
                    TopMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                    BottomMotor1.set(ControlMode.PercentOutput, BallHandlerConstants.const_ConveyerSpeedPercent);
                }
                else{
                    TopMotor1.set(ControlMode.PercentOutput, 0);
                    BottomMotor1.set(ControlMode.PercentOutput,0);
                }
            }

*/

        }

    }

    
   // public void setHoodAngle(double angle) {

    //    if (angle > BallHandlerConstants.const_HoodAngleMax){
       //     angle = BallHandlerConstants.const_HoodAngleMax;
      //  }

        

        //double setpoint = hoodangle.calculate(HoodAngleMotor.getSelectedSensorPosition(), (angle/360)* 4096 * BallHandlerConstants.const_HoodAngleRatio);

        //SmartDashboard.putNumber("Hood Setpoint", setpoint);
        

        //HoodAngleMotor.set(ControlMode.PercentOutput, setpoint);

    //}

    public void deployIntake(boolean state) {
        //IntakeDeployed++;
        if (state){
            IntakeAngleMotor.set(ControlMode.PercentOutput, .35);
        }
        else{

            IntakeAngleMotor.set(ControlMode.PercentOutput, 0);
        }

    }
    //public void stowIntake() {

      //  if (IntakeDeployed >0){
        //    IntakeAngleMotor.set(ControlMode.PercentOutput, (BallHandlerConstants.const_IntakeStowAngle/360)* 4096);
        //}
   // }

    public void resetEncoders() {
        //HoodAngleMotor.setSelectedSensorPosition((BallHandlerConstants.const_HoodAngleOffset/360)* 4096);
        //IntakeAngleMotor.setSelectedSensorPosition(0);
    }


    public void stop() {

        TopMotor1.set(ControlMode.PercentOutput, 0);
        BottomMotor1.set(ControlMode.PercentOutput,0);
        TopMotor2.set(ControlMode.PercentOutput, 0);
        BottomMotor2.set(ControlMode.PercentOutput,0);
        
        IntakeDriveMotor.set(ControlMode.PercentOutput, 0);
        HoodDriveMotor.set(ControlMode.PercentOutput, 0);
        IntakeAngleMotor.set(ControlMode.PercentOutput, 0);
        HoodDriveMotor2.set(ControlMode.PercentOutput, 0);

    }




    
}
