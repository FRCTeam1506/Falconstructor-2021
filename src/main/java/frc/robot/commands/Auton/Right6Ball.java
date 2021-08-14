package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.FastRamsete;
import frc.robot.commands.Drivetrain.StandardRamsete;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.TurnToAngleProfiled;
import frc.robot.commands.HorizIndexer.StopHorizIndexer;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.Retract;
import frc.robot.commands.Intake.StopIntakeIntake;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.VertIndexer.StopVertIndexer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class Right6Ball extends SequentialCommandGroup {
    public Right6Ball(Drivetrain drivetrain, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            // ? Drive backwards to get better angle for shooting
            new TankDrive(drivetrain, -0.5, -0.5).withTimeout(1.00),

            // ? Align and shoot
            new ParallelCommandGroup(
                new Align(drivetrain, 0.5).withTimeout(3.0),
                new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 2.0, 20000.0)
            ).withTimeout(5.5),

            // ? Stop Shooting process
            new ParallelCommandGroup(
                new StopHorizIndexer(horizIndexer),
                new StopVertIndexer(vertIndexer),
                new StopShooter(shooter),
                new StopIntakeIntake(intake)
            ).withTimeout(0.01),

            // ? Reset odometry for running path route
            new InstantCommand(() -> drivetrain.resetOdometry(RobotContainer.Six_Ball_1.getInitialPose()), drivetrain),

            // ? Run intake and drive path
            new ParallelCommandGroup(
                new ExtendAndIntake(intake).withTimeout(5.0),
                new StandardRamsete(drivetrain, RobotContainer.Six_Ball_1)
            ).withTimeout(7.0),

            // ? Retract intake
            new Retract(intake).withTimeout(0.01),

            // ? Turn around
            // new TurnToAngleProfiled(drivetrain, -180).withTimeout(3.0),

            // new PrintCommand("Made it ................."),
            // System.out.println("Made it ..............................."),

            // ? Reset odometry for running path route
            new InstantCommand(() -> drivetrain.resetOdometry(RobotContainer.Six_Ball_2.getInitialPose()), drivetrain),

            // ? Drive path
            new FastRamsete(drivetrain, RobotContainer.Six_Ball_2),

            // ? Align and shoot
            new ParallelCommandGroup(
                new Align(drivetrain, 0.5),
                new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 2.0, 20000.0)
            ).withTimeout(5.5)
        );
    }
}
