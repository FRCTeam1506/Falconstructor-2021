package frc.robot.commands.Macros.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climber.Extend;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class First extends ParallelCommandGroup {

    public First(Climber climber, Drivetrain drivetrain, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            new Extend(climber),
            new Align(drivetrain).withTimeout(3.0),
            new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 1.5, 25000.0) // 21
        );
    }

}