// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtAmpAndDriveWithControllerCommand extends AimAndDriveWithControllerCommand {

  public AimAtAmpAndDriveWithControllerCommand(DriveSubsystem drivetrain, XboxController controller) {
    super(drivetrain, controller, 6);
  }

  @Override
  public void initialize() {
    super.initialize();
    switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
      case Red:
        m_fiducialId = 5;
        break;
      case Blue:
        m_fiducialId = 6;
        break;
    }
  }

}
