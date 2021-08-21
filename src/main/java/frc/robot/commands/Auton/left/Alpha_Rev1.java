package frc.robot.commands.auton.left;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.FastRamsete;
import frc.robot.commands.Drivetrain.StandardRamsete;
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

/*
    * The main revision of th trajectory auton group codenamed `Argentina`
*/

public class Alpha_Rev1 extends SequentialCommandGroup {

    public Alpha_Rev1(Drivetrain drivetrain, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter, Trajectory firstTrajectory, Trajectory secondTrajectory, Trajectory thirdTrajectory) {
        super(

            // ? Drive backwards to get better angle for shooting
            new InstantCommand(() -> drivetrain.resetOdometry(firstTrajectory.getInitialPose()), drivetrain),
            new FastRamsete(drivetrain, firstTrajectory),

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
            new InstantCommand(() -> drivetrain.resetOdometry(secondTrajectory.getInitialPose()), drivetrain),

            // ? Run intake and drive path
            new ParallelCommandGroup(
                new ExtendAndIntake(intake).withTimeout(5.0),
                new StandardRamsete(drivetrain, secondTrajectory)
            ).withTimeout(7.0),

            // ? Retract intake
            new Retract(intake).withTimeout(0.01),

            // ? Reset odometry for running path route and run path
            new InstantCommand(() -> drivetrain.resetOdometry(thirdTrajectory.getInitialPose()), drivetrain),
            new FastRamsete(drivetrain, thirdTrajectory),

            // ? Align and shoot
            new ParallelCommandGroup(
                new Align(drivetrain, 0.5),
                new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 2.0, 20000.0)
            ).withTimeout(5.5)
        );
    }
}