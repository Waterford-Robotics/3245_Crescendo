// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PreferenceKeys;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToFiducialAndDriveWithControllerCommand extends Command {

  private final DriveSubsystem m_drivetrain;

  private final XboxController m_manualController;
  private final PIDController m_turnController;

  protected int m_fiducialId;

  /** Creates a new TurnToFiducialAndDriveWithController. */
  public TurnToFiducialAndDriveWithControllerCommand(DriveSubsystem drivetrain, XboxController controller, int fiducialId) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_manualController = controller;
    m_fiducialId = fiducialId;

    m_turnController = new PIDController(Preferences.getDouble(PreferenceKeys.kAutomaticTurningP, PIDConstants.kDefaultAutomaticTurningP),
                                         Preferences.getDouble(PreferenceKeys.kAutomaticTurningI, PIDConstants.kDefaultAutomaticTurningI),
                                         Preferences.getDouble(PreferenceKeys.kAutomaticTurningD, PIDConstants.kDefaultAutomaticTurningD));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnController.setPID(Preferences.getDouble(PreferenceKeys.kAutomaticTurningP, PIDConstants.kDefaultAutomaticTurningP),
                            Preferences.getDouble(PreferenceKeys.kAutomaticTurningI, PIDConstants.kDefaultAutomaticTurningI),
                            Preferences.getDouble(PreferenceKeys.kAutomaticTurningD, PIDConstants.kDefaultAutomaticTurningD));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleErrorDegrees = getError();

    m_drivetrain.drive(-MathUtil.applyDeadband(-m_manualController.getLeftY(), ControllerConstants.kDriveDeadband),
                       -MathUtil.applyDeadband(-m_manualController.getLeftX(), ControllerConstants.kDriveDeadband),
                       m_turnController.calculate(angleErrorDegrees),
                       true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getError() {
    Pose2d robotPose = m_drivetrain.getPose();
    Pose2d targetPose = VisionConstants.kAprilTagFieldLayout.getTagPose(m_fiducialId)
        .orElseThrow(() -> new IllegalArgumentException("The tag you requested (ID " + m_fiducialId + ") is not on the field"))
        .toPose2d();

    return -targetPose.relativeTo(robotPose).getRotation().getDegrees();
  }
}
