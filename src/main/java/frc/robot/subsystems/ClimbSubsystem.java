// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;

public class ClimbSubsystem extends SubsystemBase {
    //init stuff
      CANSparkMax climbMotor;
  
  public ClimbSubsystem() {
    //motors/encoders
      climbMotor = new CANSparkMax(MotorIDConstants.climbMotorID, MotorType.kBrushless);
    //config PID
      
    //config max output, safety
     
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
  }

  public void climbSet(){
    climbMotor.set(MotorSpeedsConstants.climbMaxVal);
  }

  public void manual(CommandXboxController controller){
    climbMotor.set(controller.getHID().getRawAxis(ControllerConstants.climbAxis));
  }

}
