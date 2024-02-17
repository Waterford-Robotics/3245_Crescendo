// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
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
    
  public IntakeSubsystem() {
    //now falcons
    //motors/encoders
      intakeBottomMotor = new CANSparkMax(MotorIDConstants.intakeBottomMotorID, MotorType.kBrushless);
      intakeTop1Motor = new CANSparkMax(MotorIDConstants.intakeTop1MotorID, MotorType.kBrushless);
      intakeTop2Motor = new CANSparkMax(MotorIDConstants.intakeTop2MotorID, MotorType.kBrushless);
      beamBreak = new DigitalInput(SensorConstants.intakeBeamBreakDIOPort);

      intakeTop2Motor.follow(intakeTop1Motor, true);    

      intakeBottomMotor.setOpenLoopRampRate(0.4);
      intakeTop1Motor.setOpenLoopRampRate(0.4);
      intakeTop2Motor.setOpenLoopRampRate(0.4);
      
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putBoolean("beam break tripped", !beamBreak.get());
  }

  public void stop(){
    intakeBottomMotor.set(0);
    intakeTop1Motor.set(0);
  }

  public void intake(){
    intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void feed(){
    intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
  }

  public void setOut(){
    intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void stopBottom(){
    intakeBottomMotor.set(0);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void manual(CommandXboxController controller){
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed*controller.getHID().getRawAxis(ControllerConstants.intakeTopAxis));
    intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoSpeed*-controller.getHID().getRawAxis(ControllerConstants.intakeBottomAxis));
  }

  public boolean getBBTripped(){
    return !beamBreak.get();
  }

}
