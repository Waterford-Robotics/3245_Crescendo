// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
      TalonFX shoulderMaster;
      TalonFX shoulderFollower;
      //TalonFXSensorCollection shoulderFalconEnc;
      CANcoder canCoder; 
      DigitalInput hallEffect;
      //TalonFXConfiguration config;

      TalonFX shoulderMaster6;
      TalonFX shoulderFollower6;
      TalonFXConfiguration falconConfig;
      Slot0Configs slot0Configs;
      CANcoderConfiguration canCoderConfig;
      PositionDutyCycle positionDutyCycle;
      VoltageOut voltage;
      //CANcoderConfigurator canCoderConfig;
    
  
  public ShoulderSubsystem() {
    //motors/encoders/pidcontroller
      shoulderFollower = new TalonFX(MotorIDConstants.shoulder2MotorID);
      shoulderMaster = new TalonFX(MotorIDConstants.shoulder3MotorID);

      //shoulderFalconEnc = new TalonFXSensorCollection(shoulderMaster);

    //CANCoder 
      falconConfig = new TalonFXConfiguration();
      canCoder = new CANcoder(MotorIDConstants.shoulderCANCoderID);
      falconConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
      falconConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    //other falcon configs
      falconConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorSpeedsConstants.shoulderRampRate;
      falconConfig.MotorOutput.PeakForwardDutyCycle = MotorSpeedsConstants.shoulderClosedMaxSpeed;
      falconConfig.MotorOutput.PeakReverseDutyCycle = -MotorSpeedsConstants.shoulderClosedMaxSpeed;
      falconConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      falconConfig.CurrentLimits.SupplyCurrentLimit = 40;

    //pid
      slot0Configs = new Slot0Configs();
      slot0Configs.kP = PIDConstants.shoulderkP;
      slot0Configs.kI = PIDConstants.shoulderkI;
      slot0Configs.kD = PIDConstants.shoulderkD;

      hallEffect = new DigitalInput(SensorConstants.hallEffectDIOPort);

    //apply configs, inversion, control requests
      shoulderMaster.getConfigurator().apply(falconConfig);
      shoulderFollower.getConfigurator().apply(falconConfig);
      shoulderMaster.getConfigurator().apply(slot0Configs, 0.05);

      shoulderMaster.setInverted(false);
      shoulderFollower.setControl(new Follower(shoulderMaster.getDeviceID(), true));

      positionDutyCycle = new PositionDutyCycle(0, 0, false, 0, 0, 
        false, false, false);
      
      voltage = new VoltageOut(0, false, false, false, false);

      }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putNumber("Shoulder Encoder Value:", canCoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("hall effect tripped", !hallEffect.get());
    if(!hallEffect.get()){
      //shoulderFalconEnc.setIntegratedSensorPosition(0.0, 15);
      //resetEncoder();
    }  }

  public void resetEncoder(){
    canCoder.setPosition(0);
  }

  public void setHome(){
    //shoulderPID.setReference(PositionValueConstants.shoulderHomePos, CANSparkMax.ControlType.kPosition);
    //shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderHomePos);
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderHomePos));

  }

  public void setAmpShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderAmpShotPos, CANSparkMax.ControlType.kPosition);
    //shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderAmpShotPos);
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderAmpShotPos));
  }

  public void setProtShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderProtShotPos, CANSparkMax.ControlType.kPosition);
    //shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderProtShotPos);
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderProtShotPos));
  }

  public void manual(CommandXboxController controller){
    //shoulderMasterMotor.set(-0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
    //shoulderMaster.set(TalonFXControlMode.PercentOutput, 0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
    //shoulderFalcon1.setControl(m_voltageRequest.withOutput(12*0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis)));
    shoulderMaster.setControl(voltage.withOutput(12*0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis)));
  }
 

}
