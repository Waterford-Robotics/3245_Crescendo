package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TeleopTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class InShooterCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    CommandXboxController m_controller;
    Timer timer = new Timer();

    public InShooterCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, CommandXboxController controller){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_controller = controller;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        m_intakeSubsystem.stop();
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    }
        
    @Override
    public void execute() {
        if(timer.get()>0.15){
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
        m_intakeSubsystem.stop();
        m_indexerSubsystem.runBackSlow();
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_intakeSubsystem.stop();
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(TeleopTimeConstants.handoffIndexerReverseTime);
    }

}