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
      TalonFX shoulderMaster;
      TalonFX shoulderFollower;
      TalonFXSensorCollection shoulderFalconEnc;
      //CANCoder canCoder; 
      public double desAngle = 0;
      public boolean spinUpAfter = false;
      DigitalInput hallEffect;
    
  
  public ShoulderSubsystem() {
    //motors/encoders/pidcontroller
      
      shoulderFollower = new TalonFX(MotorIDConstants.shoulder2MotorID);
      shoulderMaster = new TalonFX(MotorIDConstants.shoulder3MotorID);

      shoulderFalconEnc = new TalonFXSensorCollection(shoulderMaster);
      //canCoder = new CANCoder(MotorIDConstants.shoulderCANCoderID);
      //shoulderMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 15);
      //shoulderMaster.configRemoteFeedbackFilter(canCoder, 0);

      shoulderMaster.config_kF(0, 0);
      shoulderMaster.config_kP(0, PIDConstants.shoulderkP);
      shoulderMaster.config_kI(0, PIDConstants.shoulderkI);
      shoulderMaster.config_kD(0, PIDConstants.shoulderkD);

      shoulderMaster.configClosedLoopPeakOutput(0, MotorSpeedsConstants.shoulderClosedMaxSpeed);
      shoulderMaster.configPeakOutputForward(MotorSpeedsConstants.shoulderOpenMaxSpeed);
      shoulderMaster.configPeakOutputReverse(-MotorSpeedsConstants.shoulderOpenMaxSpeed);

      shoulderMaster.setInverted(InvertType.InvertMotorOutput);
      shoulderFollower.setInverted(InvertType.FollowMaster);

      shoulderFollower.follow(shoulderMaster);

      hallEffect = new DigitalInput(SensorConstants.hallEffectDIOPort);
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putNumber("Shoulder Encoder Value:", shoulderFalconEnc.getIntegratedSensorPosition());
    SmartDashboard.putBoolean("hall effect tripped", !hallEffect.get());
    if(!hallEffect.get()){
      //shoulderFalconEnc.setIntegratedSensorPosition(0.0, 15);
      shoulderMaster.setSelectedSensorPosition(0, 0, 15);
    }
  }

  public void resetEncoder(){
  }

  public void setHome(){
    //shoulderPID.setReference(PositionValueConstants.shoulderHomePos, CANSparkMax.ControlType.kPosition);
    shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderHomePos);

  }

  public void setAmpShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderAmpShotPos, CANSparkMax.ControlType.kPosition);
    shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderAmpShotPos);
  }

  public void setProtShot(){
    //shoulderPID.setReference(PositionValueConstants.shoulderProtShotPos, CANSparkMax.ControlType.kPosition);
    shoulderMaster.set(TalonFXControlMode.Position, PositionValueConstants.shoulderProtShotPos);
  }

  public void manual(CommandXboxController controller){
    //shoulderMasterMotor.set(-0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
    shoulderMaster.set(TalonFXControlMode.PercentOutput, 0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis));
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
