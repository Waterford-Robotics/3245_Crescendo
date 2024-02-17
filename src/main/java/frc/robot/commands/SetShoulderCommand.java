package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShoulderSubsystem;

public class SetShoulderCommand extends Command{
    //instantiate stuff
    ShoulderSubsystem m_shoulderSubsystem;
    String m_angle;
    //XboxController m_defaultController;

    public SetShoulderCommand(ShoulderSubsystem shoulderSubsystem, String angle/*, CommandXboxController defaultController*/){
        //definitions and setting parameters equal to members
        m_shoulderSubsystem = shoulderSubsystem;
        m_angle = angle;
        //m_defaultController = defaultController;
        addRequirements(m_shoulderSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {

        if(m_angle == "amp"){
            m_shoulderSubsystem.setAmpShot();
        }

        if(m_angle == "protected"){
            m_shoulderSubsystem.setProtShot();
        }

        if(m_angle == "home"){
            m_shoulderSubsystem.setHome();
        }

        /*else{
            new RunCommand(() -> m_shoulderSubsystem.manual(m_defaultController), m_shoulderSubsystem);
        }*/
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }

}