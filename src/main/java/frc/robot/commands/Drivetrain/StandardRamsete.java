package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class StandardRamsete extends SequentialCommandGroup {

    public StandardRamsete(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(2.0, 0.7), // 2.3
                new SimpleMotorFeedforward(
                    Constants.Drivetrain.kS,
                    Constants.Drivetrain.kV,
                    Constants.Drivetrain.kA
                ),
                Constants.Drivetrain.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(1.5, 0.01, 0.15),
                new PIDController(1.55, 0.047, 0.15),
                drivetrain::tankDriveVolts,
                drivetrain
            )
        );
    }
    
}
