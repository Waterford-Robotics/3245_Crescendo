package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CheckOrangeTrippedCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    public CheckOrangeTrippedCommand(IntakeSubsystem intakeSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        if(m_intakeSubsystem.getOverRedThresh()){
            m_intakeSubsystem.stopBottom();
        }
        else{
            m_intakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}