package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizIndexer;

public class HorizIndexer_Stop extends CommandBase {
    
    private final HorizIndexer m_horizIndexer;

    public HorizIndexer_Stop(HorizIndexer horizIndexer) {
        this.m_horizIndexer = horizIndexer;
        addRequirements(horizIndexer);
    }

    @Override
    public void execute() {
        this.m_horizIndexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
