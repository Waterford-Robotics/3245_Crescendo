package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexUntilTrippedCommand extends Command{
    //instantiate stuff
    IndexerSubsystem m_indexerSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    public IndexUntilTrippedCommand(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem){
        //definitions and setting parameters equal to members
        m_indexerSubsystem = indexerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_indexerSubsystem);
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_indexerSubsystem.feed();
        m_intakeSubsystem.feed();
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return m_indexerSubsystem.getTripped();
    }

}