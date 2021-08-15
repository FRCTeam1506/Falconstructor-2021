package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Macros.Index;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class MiddleSimple extends SequentialCommandGroup {

    public MiddleSimple(Drivetrain drivetrain, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            new TankDrive(drivetrain, -0.25, -0.27).withTimeout(0.78),
            new ParallelCommandGroup(
                new Shoot(shooter, 23000.0),
                new SequentialCommandGroup(
                    new Align(drivetrain, 1.0).withTimeout(3.0),
                    new Index(horizIndexer, vertIndexer, intake)
                )
            )
        );
    }
}