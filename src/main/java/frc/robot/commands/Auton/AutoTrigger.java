package frc.robot.commands.Auton;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class AutoTrigger extends CommandBase {
    
    private Supplier<Pose2d> drivetrainPose;
    private Intake intake;

    public AutoTrigger(Supplier<Pose2d> drivetrainPose, Intake intake) {
        this.drivetrainPose = drivetrainPose;
        this.intake = intake;
        addRequirements(intake);
    }

    private static double roundPoseValue(double n) {
        DecimalFormat df = new DecimalFormat("#.###");
        df.setRoundingMode(RoundingMode.CEILING);
        String s = df.format(n);

        return Double.valueOf(s);
    }

    @Override
    public void execute() {
        Pose2d trigger1 = new Pose2d(4.918, -0.786, new Rotation2d());
        
        if (roundPoseValue(drivetrainPose.get().getX()) == trigger1.getX()) {
            this.intake.extend();
            System.out.println("### EXTENDED ###");
        }

        // System.out.println(drivetrainPose.get().getX());
    }
}
