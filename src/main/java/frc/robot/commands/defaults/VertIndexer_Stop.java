package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VertIndexer;

public class VertIndexer_Stop extends CommandBase {
    
    private final VertIndexer m_vertIndexer;

    public VertIndexer_Stop(VertIndexer vertIndexer) {
        this.m_vertIndexer = vertIndexer;
        addRequirements(vertIndexer);
    }

    @Override
    public void execute() {
        this.m_vertIndexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
