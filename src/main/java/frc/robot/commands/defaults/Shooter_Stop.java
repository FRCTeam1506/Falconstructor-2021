package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shooter_Stop extends CommandBase {
    
    private final Shooter m_shooter;

    public Shooter_Stop(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        this.m_shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
