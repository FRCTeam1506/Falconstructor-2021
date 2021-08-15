package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Intake_RetractAndStop extends CommandBase {

    private final Intake m_intake;

    public Intake_RetractAndStop(Intake intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.m_intake.retract();
    }

    @Override
    public void execute() {
        this.m_intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
