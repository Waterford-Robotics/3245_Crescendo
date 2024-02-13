package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpinUpShootCommand extends Command{
    //instantiate stuff
    ShootSubsystem m_shootSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    Timer timer = new Timer();

    public SpinUpShootCommand(ShootSubsystem shootSubsystem, IndexerSubsystem indexerSubsystem){
        //definitions and setting parameters equal to members
        m_shootSubsystem = shootSubsystem;
        addRequirements(m_shootSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }
        
    @Override
    public void execute() {
        if(timer.get()<AutoTimeConstants.spinUpAutoTime){
            m_shootSubsystem.spinUp();
        }
        else if(timer.get()>AutoTimeConstants.spinUpAutoTime && timer.get()<AutoTimeConstants.indexAutoTime){
            m_shootSubsystem.stop();
            m_indexerSubsystem.runFast();
        }
        else if (timer.get()>AutoTimeConstants.indexAutoTime){
            m_indexerSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }

}