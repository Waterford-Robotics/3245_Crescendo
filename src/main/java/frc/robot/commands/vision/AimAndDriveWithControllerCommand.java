// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Objects;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AimAndDriveWithControllerCommand extends Command {

  private static final record AngleDistancePair (double angleErrorDegrees, double distanceToTargetMeters) {
    AngleDistancePair {
      Objects.requireNonNull(angleErrorDegrees);
      Objects.requireNonNull(distanceToTargetMeters);
    }
  }

  private static final double kTurningP = 0.01;
  private static final double kTurningI = 0.00;
  private static final double kTurningD = 0.00;

  private DriveSubsystem m_drivetrain;

  private XboxController m_manualController;
  protected int m_fiducialId;

  private PIDController m_turnController = new PIDController(kTurningP, kTurningI, kTurningD);

  /** Creates a new AimAndDriveCommand. */
  public AimAndDriveWithControllerCommand(DriveSubsystem drivetrain, XboxController controller, int targetFiducialId) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_manualController = controller;
    m_fiducialId = targetFiducialId;

    m_turnController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AngleDistancePair error = getError();
    
    m_drivetrain.drive(-MathUtil.applyDeadband(m_manualController.getLeftY(), ControllerConstants.kDriveDeadband),
                       -MathUtil.applyDeadband(m_manualController.getLeftX(), ControllerConstants.kDriveDeadband),
                       m_turnController.calculate(error.angleErrorDegrees()),
                       true, true);

    SmartDashboard.putNumber("Angle error from target (degrees)", error.angleErrorDegrees());
    SmartDashboard.putNumber("Distance from target (meters)", error.distanceToTargetMeters());
  }

  public AngleDistancePair getError() {
    Pose2d robotPose = m_drivetrain.getPose();
    Pose2d targetPose = m_drivetrain.getVisionDataProvider().getFieldLayout().getTagPose(m_fiducialId)
        .orElseThrow(() -> new IllegalArgumentException("The tag you requested (ID " + m_fiducialId + ") is not on the field"))
        .toPose2d();
    return new AngleDistancePair(-PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(),
                                  PhotonUtils.getDistanceToPose(robotPose, targetPose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
