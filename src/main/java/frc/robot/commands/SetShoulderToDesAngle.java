package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class SetShoulderToDesAngle extends Command{
    //instantiate stuff
    ShoulderSubsystem m_shoulderSubsystem;
    String m_angle;

    public SetShoulderToDesAngle(ShoulderSubsystem shoulderSubsystem, String angle){
        //definitions and setting parameters equal to members
        m_shoulderSubsystem = shoulderSubsystem;
        m_angle = angle;
        addRequirements(m_shoulderSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        m_shoulderSubsystem.setAtDesAngle();
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }

}