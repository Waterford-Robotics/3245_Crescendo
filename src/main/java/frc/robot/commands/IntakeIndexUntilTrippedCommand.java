package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeIndexUntilTrippedCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    public IntakeIndexUntilTrippedCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        m_intakeSubsystem.intake();
        m_indexerSubsystem.runFast();
    }

    @Override
    public void end(boolean interrupted){
        m_intakeSubsystem.stop();
        m_indexerSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        //return m_intakeSubsystem.getOverRedThresh();
        return m_intakeSubsystem.getBBTripped();
    }

}