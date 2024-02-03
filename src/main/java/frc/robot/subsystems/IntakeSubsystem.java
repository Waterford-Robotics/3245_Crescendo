// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.SensorConstants;

public class IntakeSubsystem extends SubsystemBase {
    //init stuff
      CANSparkMax intakeNeoMotor;
      TalonFX intakeFalconMotor;
      DigitalInput beamBreak;
  
  public IntakeSubsystem() {
    //now falcons
    //motors/encoders
      intakeNeoMotor = new CANSparkMax(MotorIDConstants.intakeNeoMotorID, MotorType.kBrushless);
      intakeFalconMotor = new TalonFX(MotorIDConstants.intakeFalconMotorID);
      beamBreak = new DigitalInput(SensorConstants.intakeBeamBreakDIOPort);
    //config PID
 
    //config max output, safety
    
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
  }

  public void stop(){
    intakeFalconMotor.set(ControlMode.PercentOutput, 0);
    intakeNeoMotor.set(0);
  }

  public void intake(){
    intakeNeoMotor.set(MotorSpeedsConstants.intakeNeoSpeed);
    intakeFalconMotor.set(ControlMode.PercentOutput, MotorSpeedsConstants.intakeFalconSpeed);
  }

  public void feed(){
    intakeNeoMotor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
    intakeFalconMotor.set(ControlMode.PercentOutput, MotorSpeedsConstants.intakeFalconFeedSpeed);
  }

  public void setOut(){
    //intakeMotor.set(-MotorSpeedsConstants.intakeSpeed);
  }

  public void manual(CommandXboxController controller){
    if(controller.getHID().getRawAxis(ControllerConstants.intakeInAxis)>0.05){
        intakeNeoMotor.set(MotorSpeedsConstants.intakeNeoSpeed);
        intakeFalconMotor.set(ControlMode.PercentOutput, MotorSpeedsConstants.intakeFalconSpeed);
    }
    else if(controller.getHID().getRawAxis(ControllerConstants.intakeOutAxis)>0.05){
        intakeNeoMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
        intakeFalconMotor.set(ControlMode.PercentOutput, -MotorSpeedsConstants.intakeFalconSpeed);
    }
    else{
        intakeNeoMotor.set(0);
    }
  }

  public boolean getTripped(){
    return beamBreak.get();
  }

}
