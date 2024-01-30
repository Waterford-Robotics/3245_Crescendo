package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUntilTrippedCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    public IntakeUntilTrippedCommand(IntakeSubsystem intakeSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        m_intakeSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.getTripped();
    }

}