// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.IndexShootCommand;
import frc.robot.commands.IndexUntilTrippedCommand;
import frc.robot.commands.IntakeUntilTrippedCommand;
import frc.robot.commands.SetShoulderCommand;
import frc.robot.commands.autos.AutoBase;
import frc.robot.commands.autos.AutonomousChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
 // private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  //private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  //private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
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

           // m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.manual(m_driverController), m_intakeSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    autoChooser.addOption("Test Auto", m_robotDrive.getAuto("Test Auto"));
        
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController.getHID(), ControllerConstants.setXValue)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

   /* SmartDashboard.putData("Auto Mode", autoChooser);
    AutoBuilder.configureHolonomic(m_robotDrive::getPose, m_robotDrive::resetOdometry, m_robotDrive::getChassisSpeeds, m_robotDrive::setChassisSpeeds,
    new HolonomicPathFollowerConfig(3, 3, new ReplanningConfig()), () -> {
                return false;
            }, m_robotDrive);*/
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
    /*SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    AutoBuilder.configureHolonomic(m_robotDrive::getPose, m_robotDrive::resetOdometry, m_robotDrive::getChassisSpeeds, m_robotDrive::setChassisSpeeds,
    new HolonomicPathFollowerConfig(3, 3, new ReplanningConfig()), () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, m_robotDrive);*/

        //SmartDashboard.putData("Test Auto", new PathPlannerAuto("Test Auto"));

    // Create config for trajectory
    //base auto command
    PIDController thetaController = new PIDController(0.05, 0, 0.05);
    PIDController xController = new PIDController(0.04, 0, 0.25);
    PIDController yController = new PIDController(0.05, 0, 0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    PathPlannerPath autoTrajectory = PathPlannerPath.fromPathFile("Trying My Best");

    /*FollowPathCommand swerveControllerCommand2 = new FollowPathCommand(new, null, null, null, null, null, null, null)
    (
          autoTrajectory, m_robotDrive::getPose,  xController,  yController, 
          thetaController, m_robotDrive::setChassisSpeeds, m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(autoTrajectory.getInitialPose());

    return swerveControllerCommand2;*/
    //return autoChooser.getSelected();
    //return auto1;
    PIDConstants translationalPID = new PIDConstants(0.15, 0, 0.05);
    PIDConstants rotPID = new PIDConstants(0.4, 0, .05);
    FollowPathHolonomic command = new FollowPathHolonomic(autoTrajectory, m_robotDrive::getPose, m_robotDrive::getChassisSpeeds, 
            m_robotDrive::setChassisSpeeds, new HolonomicPathFollowerConfig(
                translationalPID, rotPID, 3, 0.9, new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, m_robotDrive);


            //return command;

            //return AutoBuilder.followPath(autoTrajectory);
           /* FollowPathCommand command = new FollowPathCommand(autoTrajectory, m_robotDrive::getPose, m_robotDrive::getChassisSpeeds, 
            m_robotDrive::setChassisSpeeds, null, new ReplanningConfig(), () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, m_robotDrive); */
            //return command;

            return autoChooser.getSelected();
        } 

  }

