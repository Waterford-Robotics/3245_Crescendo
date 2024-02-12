package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TeleopTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHandoffCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    CommandXboxController m_controller;
    public IntakeHandoffCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, CommandXboxController controller){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_controller = controller;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {
    }
        
    @Override
    public void execute() {
        m_intakeSubsystem.intake();
        m_indexerSubsystem.runFast();

          if (m_intakeSubsystem.getOverRedThresh()) {
            m_intakeSubsystem.stopBottom();
        }

          if (m_intakeSubsystem.getBBTripped()){ 
            m_indexerSubsystem.runBackSlow();
            m_intakeSubsystem.stop();

          }
        
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

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}