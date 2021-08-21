package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class Left6Ball extends SequentialCommandGroup {
    public Left6Ball(Drivetrain drivetrain, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter, Trajectory firstTrajectory, Trajectory secondTrajectory, Trajectory thirdTrajectory) {
        super(
            
        );
    }
}
