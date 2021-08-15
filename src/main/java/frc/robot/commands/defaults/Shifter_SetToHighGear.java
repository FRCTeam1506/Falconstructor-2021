package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class Shifter_SetToHighGear extends CommandBase {

    private final Shifter m_shifter;

    public Shifter_SetToHighGear(Shifter shifter) {
        this.m_shifter = shifter;
        addRequirements(shifter);
    }

    @Override
    public void execute() {
        this.m_shifter.setToHighGear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
