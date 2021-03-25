package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.Drivetrain.StandardRamsete;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.TrajectoryLoader;

public class RamseteTriggers extends ParallelCommandGroup {

    public RamseteTriggers(Drivetrain drivetrain, Intake intake) {
        super(
            // * Ramsete
            new StandardRamsete(drivetrain, TrajectoryLoader.loadTrajectoryFromFile("1_into_trench")),

            // * Trigger
            new AutoTrigger(drivetrain::getPose, intake)
        );
    }
    
}
