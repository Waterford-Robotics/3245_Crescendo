// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Objects;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class AimAtSpeakerCommand extends Command {

  private static final record AngleDistancePair (double angleErrorDegrees, double distanceToTargetMeters) {
    AngleDistancePair {
      Objects.requireNonNull(angleErrorDegrees);
      Objects.requireNonNull(distanceToTargetMeters);
    }
  }

  private final DriveSubsystem m_drivetrain;
  private final ShoulderSubsystem m_shoulder;

  private final XboxController m_manualController;
  private final PIDController m_turnController = new PIDController(PIDConstants.kAutomaticTurningP,
                                                                   PIDConstants.kAutomaticTurningI,
                                                                   PIDConstants.kAutomaticTurningD);

  private int m_fiducialId;

  /** Creates a new AimAtSpeakerCommand. */
  public AimAtSpeakerCommand(DriveSubsystem drivetrain, ShoulderSubsystem shoulder, XboxController controller) {
    addRequirements(drivetrain, shoulder);

    m_drivetrain = drivetrain;
    m_shoulder = shoulder;
    m_manualController = controller;
    m_fiducialId = 7;
  }

  // Called when the command is initially scheduled.
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AngleDistancePair error = getError();
    
    m_drivetrain.drive(-MathUtil.applyDeadband(m_manualController.getLeftY(), ControllerConstants.kDriveDeadband),
                       -MathUtil.applyDeadband(m_manualController.getLeftX(), ControllerConstants.kDriveDeadband),
                       m_turnController.calculate(error.angleErrorDegrees()),
                       true, true);

    // TODO: Add arm aiming
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private AngleDistancePair getError() {
    Pose2d robotPose = m_drivetrain.getPose();
    Pose2d targetPose = m_drivetrain.getVisionDataProvider().getFieldLayout().getTagPose(m_fiducialId)
        .orElseThrow(() -> new IllegalArgumentException("The tag you requested (ID " + m_fiducialId + ") is not on the field"))
        .toPose2d();
    return new AngleDistancePair(-PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(),
                                  PhotonUtils.getDistanceToPose(robotPose, targetPose));
  }

}
