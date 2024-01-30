package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TeleopTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IndexShootCommand extends Command{
    //instantiate stuff
    IndexerSubsystem m_indexerSubsystem;
    ShootSubsystem m_shootSubsystem;
    CommandXboxController m_controller;
    boolean m_rightTrigReleased;
    Timer timer;
    int count;
    public IndexShootCommand(IndexerSubsystem indexerSubsystem, ShootSubsystem shootSubsystem, CommandXboxController controller, boolean rightTrigReleased){
        //definitions and setting parameters equal to members
        m_indexerSubsystem = indexerSubsystem;
        m_shootSubsystem = shootSubsystem;
        m_controller = controller;
        m_rightTrigReleased = rightTrigReleased;
        timer = new Timer();
        addRequirements(m_indexerSubsystem);
        addRequirements(m_shootSubsystem);
    }

    @Override
    public void initialize() {
        count = 0;
    }
        
    @Override
    public void execute() {
        if(!m_rightTrigReleased){
            //when holding right trigger, spin up shooter
            m_shootSubsystem.spinUp();
            count = 1;
        }
        else if (m_rightTrigReleased && count == 1){
            //when right trigger released, shoot via indexer and stop both mechanisms after set time
            timer.start();
            timer.reset();
            if(timer.get()<TeleopTimeConstants.indexerShootSpinAfterTime){
                m_indexerSubsystem.spit();
            }
            if(timer.get()>TeleopTimeConstants.indexerShootSpinAfterTime){
                m_indexerSubsystem.stop();
                m_shootSubsystem.stop();
            }
        }

        else{
            m_indexerSubsystem.stop();
            m_shootSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_shootSubsystem.stop();
        count = 0;
    }

    @Override
    public boolean isFinished(){
        //if left trigger pressed, stop everything
        return m_controller.getHID().getRawButton(ControllerConstants.indexShootStopButton);
    }

}