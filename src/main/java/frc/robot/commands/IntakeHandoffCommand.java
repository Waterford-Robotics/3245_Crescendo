package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TeleopTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHandoffCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    Timer timer = new Timer();
    public IntakeHandoffCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {
    }
        
    @Override
    public void execute() {
        
          if (m_intakeSubsystem.getOverRedThresh()) {
            m_intakeSubsystem.stopBottom();
            m_indexerSubsystem.runSlow();
        }

        else if (m_intakeSubsystem.getBBTripped()){ 
            m_indexerSubsystem.stop();
            m_intakeSubsystem.stop();
            /*timer.start();
            timer.reset();
            if(timer.get()<TeleopTimeConstants.handoffIndexerReverseTime){
                m_indexerSubsystem.runBackSlow();
                m_intakeSubsystem.stop();
            }
            else{
                m_indexerSubsystem.stop();
            }*/
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_intakeSubsystem.stop();
        timer.stop(); 
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}