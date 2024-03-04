// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToSpeakerAndDriveWithControllerCommand extends TurnToFiducialAndDriveWithControllerCommand {
  
  public TurnToSpeakerAndDriveWithControllerCommand(DriveSubsystem drivetrain, XboxController controller) {
    super(drivetrain, controller, 7);
  }

  @Override
  public void initialize() {
    super.initialize();
    switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
      case Red:
        m_fiducialId = 4;
        break;
      case Blue:
      default:
        m_fiducialId = 7;
        break;
    }
  }

}
