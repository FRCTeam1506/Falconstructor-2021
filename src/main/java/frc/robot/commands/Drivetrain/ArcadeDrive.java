package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final Shifter m_shifter;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public ArcadeDrive(Drivetrain drivetrain, Shifter shifter, DoubleSupplier fwd, DoubleSupplier rot) {
    m_drivetrain = drivetrain;
    m_shifter = shifter;
    m_forward = fwd;
    m_rotation = rot;
    addRequirements(m_drivetrain, m_shifter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_shifter.getState() == "Low") {
      m_drivetrain.lowGearArcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
    } else {
      m_drivetrain.highGearArcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
