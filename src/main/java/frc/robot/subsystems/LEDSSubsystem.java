// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class LEDSSubsystem extends SubsystemBase {
    //init stuff
    Spark blinkin;

  public LEDSSubsystem() {
    //motors/encoders
        blinkin = new Spark(MotorIDConstants.blinkinPort);
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    
  }

  public void setRed(){}

  public void setBlue(){}

  public void setGreen(){}

  public void setYellow(){}

  public void setPurple(){}

  public void setRainbow(){}

  public void setSparkles(){}

  public void setStrobe(){}
}
