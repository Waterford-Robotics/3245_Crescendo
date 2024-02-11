// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PositionValueConstants;
import frc.robot.Constants.SensorConstants;

public class ShoulderSubsystem extends SubsystemBase {
    //init stuff
      CANSparkMax shoulderMasterMotor;
      //CANSparkMax shoulderFollower1Motor;
      CANSparkMax shoulderFollower2Motor;
      CANSparkMax shoulderFollower3Motor;
      SparkMaxPIDController shoulderPID;
      AbsoluteEncoder shoulderEncoder;
      public double desAngle = 0;
      public boolean spinUpAfter = false;
  
  public ShoulderSubsystem() {
    //motors/encoders/pidcontroller
      shoulderMasterMotor = new CANSparkMax(MotorIDConstants.shoulder1MotorID, MotorType.kBrushless);
      //shoulderFollower1Motor = new CANSparkMax(MotorIDConstants.shoulder2MotorID, MotorType.kBrushless);
      shoulderFollower2Motor = new CANSparkMax(MotorIDConstants.shoulder3MotorID, MotorType.kBrushless);
      shoulderFollower3Motor = new CANSparkMax(MotorIDConstants.shoulder4MotorID, MotorType.kBrushless);

      shoulderEncoder = shoulderMasterMotor.getAbsoluteEncoder(Type.kDutyCycle);
      
      shoulderPID = shoulderMasterMotor.getPIDController();


    //config PID
      shoulderPID.setFF(PIDConstants.shoulderkF);
      shoulderPID.setP(PIDConstants.shoulderkP);
      shoulderPID.setI(PIDConstants.shoulderkI);
      shoulderPID.setD(PIDConstants.shoulderkD);

      //shoulderFollower1Motor.follow(shoulderFollower2Motor);
      shoulderFollower2Motor.follow(shoulderMasterMotor);
      shoulderFollower3Motor.follow(shoulderMasterMotor);

    //config max output, safety
      shoulderMasterMotor.setOpenLoopRampRate(MotorSpeedsConstants.shoulderRampRate);
      shoulderMasterMotor.setClosedLoopRampRate(MotorSpeedsConstants.shoulderRampRate);
      shoulderPID.setOutputRange(-MotorSpeedsConstants.shoulderClosedMaxSpeed, MotorSpeedsConstants.shoulderClosedMaxSpeed);     
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putNumber("Shoulder Encoder Value:", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder Pos Conversion Factor:", shoulderEncoder.getPositionConversionFactor());



    /*if(hallEffect.get()){
        shoulderEncoder.setPosition(0);
    }*/
  }

  public void resetEncoder(){
  }

  public void setHome(){
    shoulderPID.setReference(PositionValueConstants.shoulderHomePos, CANSparkMax.ControlType.kPosition);
  }

  public void setAmpShot(){
    shoulderPID.setReference(PositionValueConstants.shoulderAmpShotPos, CANSparkMax.ControlType.kPosition);
  }

  public void setProtShot(){
    shoulderPID.setReference(PositionValueConstants.shoulderProtShotPos, CANSparkMax.ControlType.kPosition);
  }

  public void manual(CommandXboxController controller){
    shoulderMasterMotor.set(-0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
  }

  public void setDesAngle(double desiredAngle){
    desAngle = desiredAngle;
  }

  public double getDesAngle(){
    return desAngle;
  }

  public void setAtDesAngle(){
    shoulderPID.setReference(getDesAngle(), CANSparkMax.ControlType.kPosition);
  }

  public boolean getSpinUpAfter(){ 
    return spinUpAfter;
  }

  public void setSpinUpAfter(boolean doesSpinUpAfter){
    spinUpAfter = doesSpinUpAfter;
  }

}
