package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBase extends SequentialCommandGroup{
    
    DriveSubsystem m_driveSubsystem;
    String m_path;
    double m_maxVel;
    double m_maxAcc;
    public final PIDController thetaController;
    
    public AutoBase(DriveSubsystem driveSubsystem){
        thetaController = new PIDController(0.05, 0, 0.05);

        m_driveSubsystem = driveSubsystem;

        addRequirements(m_driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    }  
    
       /* public FollowPathCommand baseSwerveCommand(PathPlannerPath path, boolean isRed){
            PathFollowingController pathController = new PathFollowingController() {
                
            }
            FollowPathCommand command = new FollowPathCommand(path, m_driveSubsystem::getPose, 
            m_driveSubsystem::getChassisSpeeds, m_driveSubsystem::setChassisSpeeds, null, 
            new ReplanningConfig(), () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, m_driveSubsystem);

            return command;
        }*/

        public FollowPathHolonomic baseSwerveCommandHolonomic(PathPlannerPath path){
            FollowPathHolonomic command = new FollowPathHolonomic(path, m_driveSubsystem::getPose, m_driveSubsystem::getChassisSpeeds, 
            m_driveSubsystem::setChassisSpeeds, new HolonomicPathFollowerConfig(m_maxVel, 0.9, new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, m_driveSubsystem);

            return command;
        }
        
    }