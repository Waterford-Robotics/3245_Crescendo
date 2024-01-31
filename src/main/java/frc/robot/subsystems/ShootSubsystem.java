// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;

public class ShootSubsystem extends SubsystemBase {
    //init stuff
    TalonFX shootMotor1;
    TalonFX shootMotor2;
    MotorOutputConfigs motor1OutputConfigs;
    MotorOutputConfigs motor2OutputConfigs;
  public ShootSubsystem() {
    //motors/encoders
      shootMotor1 = new TalonFX(MotorIDConstants.shootMotor1ID);
      shootMotor2 = new TalonFX(MotorIDConstants.shootMotor2ID);
 
    //config max output, safety
      motor1OutputConfigs = new MotorOutputConfigs();
      motor2OutputConfigs = new MotorOutputConfigs();
      motor1OutputConfigs.withPeakForwardDutyCycle(MotorSpeedsConstants.shoot1MaxVal);
      motor2OutputConfigs.withPeakForwardDutyCycle(MotorSpeedsConstants.shoot2MaxVal);
      shootMotor1.getConfigurator().apply(motor1OutputConfigs);
      shootMotor2.getConfigurator().apply(motor2OutputConfigs);
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
    if(controller.getHID().getRawButton(ControllerConstants.shootButton)){
        shootMotor1.set(MotorSpeedsConstants.shoot1MaxVal);
        shootMotor2.set(MotorSpeedsConstants.shoot2MaxVal);
    }
    else if(controller.getHID().getRawButton(ControllerConstants.shootBackButton)){
        shootMotor1.set(-MotorSpeedsConstants.shoot1MaxVal);
        shootMotor2.set(-MotorSpeedsConstants.shoot2MaxVal);
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
