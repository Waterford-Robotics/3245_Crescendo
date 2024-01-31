package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestFalconIntakeSubsystem;

public class TestFalconIntakeRunForSecs extends Command{
    //instantiate stuff
    TestFalconIntakeSubsystem m_testSubsystem;
    double m_targetSecs;
    Timer timer = new Timer();

    public TestFalconIntakeRunForSecs(TestFalconIntakeSubsystem testSubsystem, double targetSecs){
        //definitions and setting parameters equal to members
        m_testSubsystem = testSubsystem;
        m_targetSecs = targetSecs;
        addRequirements(m_testSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }
        
    @Override
    public void execute() {
        m_testSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted){
        m_testSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(m_targetSecs);
    }

}