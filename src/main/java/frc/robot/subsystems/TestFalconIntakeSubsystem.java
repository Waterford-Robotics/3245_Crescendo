// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestFalconIntakeSubsystem extends SubsystemBase {
    //init stuff
      TalonFX intakeMotor;
  
  public TestFalconIntakeSubsystem() {
    //now falcons
    //motors/encoders
    intakeMotor = new TalonFX(50);
    //config PID
 
    //config max output, safety
    
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
  }

  public void stop(){
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void intake(){
    intakeMotor.set(TalonFXControlMode.Current, 0.1);
  }

  public void feed(){
  }

  public void setOut(){
  }

}
