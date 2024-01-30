// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
      CANSparkMax intakeMotor;
      DigitalInput beamBreak;
  
  public IntakeSubsystem() {
    //motors/encoders
      intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotorID, MotorType.kBrushless);
      beamBreak = new DigitalInput(SensorConstants.intakeBeamBreakDIOPort);
    //config PID
 
    //config max output, safety
    
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
  }

  public void stop(){
    intakeMotor.set(0);
  }

  public void intake(){
    intakeMotor.set(MotorSpeedsConstants.intakeSpeed);
  }

  public void feed(){
    intakeMotor.set(MotorSpeedsConstants.intakeFeedSpeed);
  }

  public void setOut(){
    intakeMotor.set(-MotorSpeedsConstants.intakeSpeed);
  }

  public void manual(CommandXboxController controller){
    if(controller.getHID().getRawAxis(ControllerConstants.intakeInAxis)>0.05){
        intakeMotor.set(MotorSpeedsConstants.intakeSpeed);
    }
    else if(controller.getHID().getRawAxis(ControllerConstants.intakeOutAxis)>0.05){
        intakeMotor.set(-MotorSpeedsConstants.intakeSpeed);
    }
    else{
        intakeMotor.set(0);
    }
  }

  public boolean getTripped(){
    return beamBreak.get();
  }

}
