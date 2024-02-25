// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.utils.VisionDataProvider;

/**
 * A command that automatically adjusts the arm and the angle of the robot's chassis to guarantee accurate shots into the Speaker.
 */
public class AimAtSpeakerAutomaticallyAndDriveWithControllerCommand extends Command {

  private static final record AngleDistancePair (double angleErrorDegrees, double distanceToTargetMeters) {
    AngleDistancePair {
      Objects.requireNonNull(angleErrorDegrees);
      Objects.requireNonNull(distanceToTargetMeters);
    }
  }

  private static final double kTurningP = 0.0;
  private static final double kTurningI = 0.0;
  private static final double kTurningD = 0.0;

  private final DriveSubsystem m_drivetrain;
  private final ShoulderSubsystem m_shoulder;

  private final XboxController m_movingController;

  private final PIDController m_turningController = new PIDController(kTurningP, kTurningI, kTurningD);

  private int m_fiducialId;

  /**
   * Creates a new AimAutomaticallyAndDriveWithControllerCommand.
   */
  public AimAtSpeakerAutomaticallyAndDriveWithControllerCommand(DriveSubsystem drivetrain, ShoulderSubsystem shoulder, XboxController controller) {
    addRequirements(drivetrain, shoulder);
    m_drivetrain = drivetrain;
    m_shoulder = shoulder;
    m_movingController = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the target fiducial to the one for your current alliance.
    switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
      case Red:
        m_fiducialId = 4;
      case Blue:
        m_fiducialId = 7;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Determine the relative angle and distance of the Speaker.
    AngleDistancePair relativePoseEstimates = getRelativeMeasurementFromSeenTarget().orElseGet(this::getRelativeMeasurementFromRobotPose);
    
    // Drive the robot. Linear movement is determined by controller input, but angular movement is determined by the
    // relative angle of the target.
    m_drivetrain.drive(-MathUtil.applyDeadband(m_movingController.getLeftY(), ControllerConstants.kDriveDeadband),
                       -MathUtil.applyDeadband(m_movingController.getLeftX(), ControllerConstants.kDriveDeadband),
                       m_turningController.calculate(relativePoseEstimates.angleErrorDegrees()),
                       true, true);
    
    // TODO: Implement arm aiming. You can use relativePoseEstimates.distanceToTargetMeters() to obtain the necessary information.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Attempt to get the estimate of relative angle and distance to the Speaker. Although this method is more accurate
   * than getting information from the robot pose, it requires the target to be detected by the camera. If not seen by
   * the camera, Optional.empty() will be returned.
   * @return An Optional wrapping an AngleDistancePair containing the angle and distance information, or empty if that
   *    information could not be determined.
   */
  private Optional<AngleDistancePair> getRelativeMeasurementFromSeenTarget() {
    Optional<PhotonTrackedTarget> target = m_drivetrain.getVisionDataProvider().getTargets().stream()
      .filter(x -> x.getFiducialId() == m_fiducialId)
      .findFirst();
    
    if (target.isPresent()) {
      PhotonTrackedTarget seenTarget = target.get();
      VisionDataProvider visionDataProvider = m_drivetrain.getVisionDataProvider();
      Transform3d robotToCameraTransform = visionDataProvider.getRobotToCameraTransform();
      Pose3d tagPose = visionDataProvider.getFieldLayout().getTagPose(m_fiducialId)
          .orElseThrow(() -> new IllegalArgumentException("The tag you requested (ID " + m_fiducialId + ") is not on the field"));

      return Optional.of(new AngleDistancePair(seenTarget.getYaw(),
                                               PhotonUtils.calculateDistanceToTargetMeters(robotToCameraTransform.getZ(),
                                                                                           tagPose.getZ(),
                                                                                           robotToCameraTransform.getRotation().getY(),
                                                                                           Units.degreesToRadians(seenTarget.getPitch()))));
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get the estimate of relative angle and distance to the Speaker using information about the robot's position on the
   * field. This method never fails, but is less accurate than when the target is seen.
   * @return An AngleDistancePair containing the angle and distance information.
   */
  private AngleDistancePair getRelativeMeasurementFromRobotPose() {
    Pose2d robotPose = m_drivetrain.getPose();
    Pose2d targetPose = m_drivetrain.getVisionDataProvider().getFieldLayout().getTagPose(m_fiducialId)
        .orElseThrow(() -> new IllegalArgumentException("The tag you requested (ID " + m_fiducialId + ") is not on the field"))
        .toPose2d();
      
    return new AngleDistancePair(-PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(),
                                 PhotonUtils.getDistanceToPose(robotPose, targetPose));
  }

}
