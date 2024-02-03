// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.IndexShootCommand;
import frc.robot.commands.IndexUntilTrippedCommand;
import frc.robot.commands.IntakeUntilTrippedCommand;
import frc.robot.commands.SetShoulderCommand;
import frc.robot.commands.TestFalconIntakeRunForSecs;
import frc.robot.commands.autos.AutoBase;
import frc.robot.commands.autos.AutonomousChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TestFalconIntakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
 // private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  //private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
  SendableChooser<Command> autoChooser;
  
  // controllers
  CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);

  // commands
 /* Command handoffCommandGroup = Commands.parallel(
      new IntakeUntilTrippedCommand(m_intakeSubsystem),
      new SetShoulderCommand(m_shoulderSubsystem, "home"))
      .andThen(new IndexUntilTrippedCommand(m_indexerSubsystem, m_intakeSubsystem))
      .andThen(new SetShoulderCommand(m_shoulderSubsystem, "high")); */

  

  // auto routines
  //Command auto1 = new AutoBase(m_robotDrive, "DriveStraightSpin", 4, 3);


  public RobotContainer() {
    m_robotDrive.calibrateGyro();
    // default commands
   /* m_shootSubsystem.setDefaultCommand(
        new RunCommand(() -> m_shootSubsystem.manual(m_operatorController), m_shootSubsystem));*/
    
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    m_shoulderSubsystem.setDefaultCommand(new RunCommand(() -> m_shoulderSubsystem.manual(m_operatorController), m_shoulderSubsystem));
    m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.manual(m_operatorController), m_intakeSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    autoChooser.addOption("Test Auto", m_robotDrive.getAuto("Test Auto"));
    autoChooser.addOption("Score 1 Speaker", m_robotDrive.getAuto("Score 1 Speaker"));
    autoChooser.addOption("follow Go To Ring 1 path", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Go To Ring 1")));
    autoChooser.addOption("go to ring 1 and intake", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Go To Ring 1")).andThen(new WaitCommand(1)).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("Ring 1 to Speaker")))); 
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController.getHID(), ControllerConstants.setXValue)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    /*new JoystickButton(m_operatorController.getHID(), ControllerConstants.indexShootButton)
        .whileTrue(new IndexShootCommand(m_indexerSubsystem, m_shootSubsystem, m_operatorController, false))
        .whileFalse(new IndexShootCommand(m_indexerSubsystem, m_shootSubsystem, m_operatorController, true));
*/
    //new Trigger(m_operatorController.axisGreaterThan(ControllerConstants.intakeAxis, 0.8)).whileTrue(handoffCommandGroup);

    /*
     * Weird stuff Niko wants:
     * A, B, X --> shoulder.setshoulderstates(each respective mode)
     * LB: if shoulder.getspinupafter, setshouldertodesangle().sequence(whatever he wants that has no spinup)
     *     else if !shoulder.getspinupafter, setshouldertodesangle().sequence(regular shooting sequence command)
     * :)
     */
  }

  public Command getAutonomousCommand() {

    // Create config for trajectory
    //base auto command

            return autoChooser.getSelected();
            //return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Please God"));
    } 

  }

