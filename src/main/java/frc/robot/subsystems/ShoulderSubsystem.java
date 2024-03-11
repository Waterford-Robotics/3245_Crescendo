// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
      TalonFX shoulderFalcon1;
      TalonFX shoulderFalcon2;
      TalonFX shoulderFalcon3;
      TalonFX shoulderFalcon4;
      TalonFXSensorCollection shoulderFalconEnc;
      CANCoder canCoder; 
      public double desAngle = 0;
      public boolean spinUpAfter = false;
      DigitalInput hallEffect;
    
  
  public ShoulderSubsystem() {
    //motors/encoders/pidcontroller
      
      shoulderFalcon1 = new TalonFX(MotorIDConstants.shoulder1MotorID);
      shoulderFalcon2 = new TalonFX(MotorIDConstants.shoulder2MotorID);
      shoulderFalcon3 = new TalonFX(MotorIDConstants.shoulder3MotorID);
      shoulderFalcon4 = new TalonFX(MotorIDConstants.shoulder4MotorID);

      shoulderFalconEnc = new TalonFXSensorCollection(shoulderFalcon1);
      canCoder = new CANCoder(MotorIDConstants.shoulderCANCoderID);
      shoulderFalcon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 15);
      shoulderFalcon1.configRemoteFeedbackFilter(canCoder, 0);

      shoulderFalcon1.config_kF(0, 0);
      shoulderFalcon1.config_kP(0, PIDConstants.shoulderkP);
      shoulderFalcon1.config_kI(0, PIDConstants.shoulderkI);
      shoulderFalcon1.config_kD(0, PIDConstants.shoulderkD);

      shoulderFalcon1.configClosedLoopPeakOutput(0, MotorSpeedsConstants.shoulderClosedMaxSpeed);
      shoulderFalcon1.configPeakOutputForward(MotorSpeedsConstants.shoulderOpenMaxSpeed);
      shoulderFalcon1.configPeakOutputReverse(-MotorSpeedsConstants.shoulderOpenMaxSpeed);

      shoulderFalcon3.setInverted(InvertType.OpposeMaster);
      shoulderFalcon2.setInverted(InvertType.OpposeMaster);

      shoulderFalcon4.follow(shoulderFalcon1);
      shoulderFalcon2.follow(shoulderFalcon1);
      shoulderFalcon3.follow(shoulderFalcon1);

      hallEffect = new DigitalInput(SensorConstants.hallEffectDIOPort);
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putNumber("Shoulder Encoder Value:", shoulderFalconEnc.getIntegratedSensorPosition());
    SmartDashboard.putBoolean("hall effect tripped", !hallEffect.get());
    if(!hallEffect.get()){
      //shoulderFalconEnc.setIntegratedSensorPosition(0.0, 15);
      shoulderFalcon1.setSelectedSensorPosition(0, 0, 15);
    }
  }

  public void resetEncoder(){
  }

  public void setHome(){
    //shoulderPID.setReference(PositionValueConstants.shoulderHomePos, CANSparkMax.ControlType.kPosition);
    shoulderFalcon1.set(TalonFXControlMode.Position, PositionValueConstants.shoulderHomePos);

  }

  public void setAmpShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderAmpShotPos, CANSparkMax.ControlType.kPosition);
    shoulderFalcon1.set(TalonFXControlMode.Position, PositionValueConstants.shoulderAmpShotPos);
  }

  public void setProtShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderProtShotPos, CANSparkMax.ControlType.kPosition);
    shoulderFalcon1.set(TalonFXControlMode.Position, PositionValueConstants.shoulderProtShotPos);
  }

  public void manual(CommandXboxController controller){
    //shoulderMasterMotor.set(-0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
    shoulderFalcon1.set(TalonFXControlMode.PercentOutput, 0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
    //shoulderFalcon1.setControl(m_voltageRequest.withOutput(12*0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis)));
  }

  public void setDesAngle(double desiredAngle){
    desAngle = desiredAngle;
  }

  public double getDesAngle(){
    return desAngle;
  }

  public void setAtDesAngle(){
  }

  public boolean getSpinUpAfter(){ 
    return spinUpAfter;
  }

  public void setSpinUpAfter(boolean doesSpinUpAfter){
    spinUpAfter = doesSpinUpAfter;
  }

}
