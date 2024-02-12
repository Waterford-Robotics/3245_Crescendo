// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;

public class ShootSubsystem extends SubsystemBase {
    //init stuff
    CANSparkFlex shootMotor1;
    CANSparkFlex shootMotor2;
  public ShootSubsystem() {
    //motors/encoders
      shootMotor1 = new CANSparkFlex(MotorIDConstants.shootMotor1ID, MotorType.kBrushless);
      shootMotor2 = new CANSparkFlex(MotorIDConstants.shootMotor2ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    
  }

  public void spinUp(){
    shootMotor1.set(MotorSpeedsConstants.shoot1MaxVal);
    shootMotor2.set(MotorSpeedsConstants.shoot2MaxVal);
  }

  public void manual(CommandXboxController controller){
    if(controller.getHID().getRawButton(ControllerConstants.spinupButton)){
        shootMotor1.set(-MotorSpeedsConstants.shoot1MaxVal);
        shootMotor2.set(MotorSpeedsConstants.shoot2MaxVal);
    }
    else{
        shootMotor1.set(0);
        shootMotor2.set(0);
    }
  }

  public void stop(){
    shootMotor1.set(0);
    shootMotor2.set(0);
  }

}
