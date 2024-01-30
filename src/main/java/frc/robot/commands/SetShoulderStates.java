package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionValueConstants;
import frc.robot.subsystems.ShoulderSubsystem;

public class SetShoulderStates extends Command{
    //instantiate stuff
    ShoulderSubsystem m_shoulderSubsystem;
    String m_angle;

    public SetShoulderStates(ShoulderSubsystem shoulderSubsystem, String angle){
        //definitions and setting parameters equal to members
        m_shoulderSubsystem = shoulderSubsystem;
        m_angle = angle;
        addRequirements(m_shoulderSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {

        if(m_angle == "amp"){
            m_shoulderSubsystem.setDesAngle(PositionValueConstants.shoulderAmpShotPos);
            m_shoulderSubsystem.setSpinUpAfter(false);
        }

        else if(m_angle == "protected"){
            m_shoulderSubsystem.setDesAngle(PositionValueConstants.shoulderProtShotPos);
            m_shoulderSubsystem.setSpinUpAfter(true);
        }

        else if(m_angle == "home"){
            m_shoulderSubsystem.setDesAngle(PositionValueConstants.shoulderHomePos);
            m_shoulderSubsystem.setSpinUpAfter(true);
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }

}