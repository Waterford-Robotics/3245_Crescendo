// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoInShooterCommand;
import frc.robot.commands.InShooterCommand;
import frc.robot.commands.IndexShootCommand;
import frc.robot.commands.IndexToShootCommand;
import frc.robot.commands.IntakeIndexUntilTrippedCommand;
import frc.robot.commands.SetShoulderCommand;
import frc.robot.commands.SpinUpAutoCommand;
import frc.robot.commands.SpinUpShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
  SendableChooser<Command> autoChooser;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // controllers
  CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);

  // commands
  SetShoulderCommand shimmyUp = new SetShoulderCommand(m_shoulderSubsystem, "protected");
  SetShoulderCommand shimmyDown = new SetShoulderCommand(m_shoulderSubsystem, "home");
  SequentialCommandGroup handoffCommand = new SequentialCommandGroup(
    new IntakeIndexUntilTrippedCommand(m_intakeSubsystem, m_indexerSubsystem, m_shootSubsystem),
   /* new IntakeIntoShooterCommand(m_intakeSubsystem, m_indexerSubsystem),*/
    new InShooterCommand(m_intakeSubsystem, m_indexerSubsystem)
  );
  SequentialCommandGroup autoHandoffCommand = new SequentialCommandGroup(
    new IntakeIndexUntilTrippedCommand(m_intakeSubsystem, m_indexerSubsystem, m_shootSubsystem),
    new AutoInShooterCommand(m_intakeSubsystem, m_indexerSubsystem)
  );

  // auto routines
  SequentialCommandGroup testSeq = new SequentialCommandGroup(
        m_robotDrive.getPath("Speaker to Ring 1"),
        new SetShoulderCommand(m_shoulderSubsystem, "protected")
  );

  public RobotContainer() {
    m_robotDrive.calibrateGyro();
    // default commands
    NamedCommands.registerCommand("Arm Up", new SetShoulderCommand(m_shoulderSubsystem, "protected"));
    NamedCommands.registerCommand("Run Intake", autoHandoffCommand);
    NamedCommands.registerCommand("Spin Up Shoot", new SpinUpShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    NamedCommands.registerCommand("Spin Up", new SpinUpAutoCommand(m_shootSubsystem));
    NamedCommands.registerCommand("Shoot", new IndexToShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    m_shootSubsystem.setDefaultCommand(new RunCommand(() -> m_shootSubsystem.manual(m_driverController), m_shootSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoMode", m_chooser);

    //auto options
    m_chooser.addOption("Score 1 wherever", new SpinUpShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    m_chooser.addOption("score 2 (ring close to wall) center", m_robotDrive.getAuto("Score 2 Center"));
    m_chooser.addOption("score 2 wall side", m_robotDrive.getAuto("Score 2 Wall Side"));
    m_chooser.addOption("score 2 field side", m_robotDrive.getAuto("Score 2 Field Side"));
    m_chooser.addOption("score 3 center", m_robotDrive.getAuto("Score 3 Center"));
    m_chooser.addOption("score 3 wall side", m_robotDrive.getAuto("Score 3 Wall Side"));
    m_chooser.addOption("score 3 field side", m_robotDrive.getAuto("Score 3 Field Side"));
    m_chooser.addOption("score 4 center", m_robotDrive.getAuto("Score 4 Center"));
    m_chooser.addOption("score 4 wall side", m_robotDrive.getAuto("Score 4 Wall Side"));
    m_chooser.addOption("score 4 field side", m_robotDrive.getAuto("Score 4 Field Side"));
    m_chooser.addOption("score 5??", m_robotDrive.getAuto("Score 5 Center"));
    m_chooser.addOption("score 2 midline", m_robotDrive.getAuto("Score 2 Midline"));
    m_chooser.addOption("experimental score 4 center", m_robotDrive.getAuto("Score 4 Center Experiment"));
    m_chooser.addOption("null auto", new WaitCommand(0.05));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController.getHID(), ControllerConstants.setXValue)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //shoulder
    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderHomeButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "home"));

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderAmpButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "amp"));

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderProtButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "protected"));


    //handoff
    new Trigger(m_driverController.axisGreaterThan(ControllerConstants.intakeAxis, 0.5)).whileTrue(
      handoffCommand
    );

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shootButton).whileTrue(
      new InstantCommand(() -> m_indexerSubsystem.runFast(), m_indexerSubsystem).alongWith(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
      )
    ).whileFalse(
      new InstantCommand(() -> m_indexerSubsystem.stop(), m_indexerSubsystem).alongWith(
        new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
      )
    );

  }

  public Command getAutonomousCommand() {
            return m_chooser.getSelected();   
    } 

  }

