// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.SensorConstants;

public class IntakeSubsystem extends SubsystemBase {
    //init stuff
      CANSparkMax intakeBottomMotor;
      CANSparkMax intakeTop1Motor;
      CANSparkMax intakeTop2Motor;
      DigitalInput beamBreak;
      ColorSensorV3 colorSensor;
      Color kOrangeTarget;
      Color detectedColor;
      ColorMatch m_colorMatcher;
      ColorMatchResult m_colorMatchResult;
  
  public IntakeSubsystem() {
    //now falcons
    //motors/encoders
      final I2C.Port i2cPort = I2C.Port.kOnboard;
      intakeBottomMotor = new CANSparkMax(MotorIDConstants.intakeBottomMotorID, MotorType.kBrushless);
      intakeTop1Motor = new CANSparkMax(MotorIDConstants.intakeTop1MotorID, MotorType.kBrushless);
      intakeTop2Motor = new CANSparkMax(MotorIDConstants.intakeTop2MotorID, MotorType.kBrushless);
      beamBreak = new DigitalInput(SensorConstants.intakeBeamBreakDIOPort);
      colorSensor = new ColorSensorV3(i2cPort);
      kOrangeTarget = new Color(255, 165, 0);
      m_colorMatcher = new ColorMatch();
 
      intakeTop2Motor.follow(intakeTop1Motor, true);    

      m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    detectedColor = colorSensor.getColor();
    m_colorMatchResult = m_colorMatcher.matchClosestColor(detectedColor);
    SmartDashboard.putBoolean("orange detected", getOrange());
  }

  public void stop(){
    intakeBottomMotor.set(0);
    intakeTop1Motor.set(0);
  }

  public void intake(){
    intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void feed(){
    intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
    //intakeFalconMotor.set(ControlMode.PercentOutput, MotorSpeedsConstants.intakeFalconFeedSpeed);
  }

  public void setOut(){
    intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void manual(CommandXboxController controller){
    /*if(controller.getHID().getRawAxis(ControllerConstants.intakeTopAxis)>0.05){
        intakeNeo2Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
    }
  
    else if(controller.getHID().getRawAxis(ControllerConstants.intakeBottomAxis)>0.05){
        intakeNeoMotor.set(MotorSpeedsConstants.intakeNeoSpeed);
        intakeNeo2Motor.set(-MotorSpeedsConstants.intakeNeoSpeed);
    }
    else{
        intakeNeoMotor.set(0);
        intakeNeo2Motor.set(0);
    }*/
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed*controller.getHID().getRawAxis(ControllerConstants.intakeTopAxis));
    intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoSpeed*-controller.getHID().getRawAxis(ControllerConstants.intakeBottomAxis));

  }
  public boolean getOrange(){
    return m_colorMatchResult.color == kOrangeTarget;
  }
  public boolean getTripped(){
    return beamBreak.get();
  }

}
