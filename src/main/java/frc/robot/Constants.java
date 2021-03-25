package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class General {}

    public static final class Auto {
        public static enum Position {
            Nothing,
            Left,
            Middle,
            Right
        }
        public static enum Goal {
            Safe,
            Ambitious,
        }
        public static enum Test {

        }
    }

    public static final class Drivetrain {
        public static final Integer LEFT_DRIVE_MASTER_ID = 0;
        public static final Integer LEFT_DRIVE_ID = 1;
        public static final Integer RIGHT_DRIVE_MASTER_ID = 14;
        public static final Integer RIGHT_DRIVE_ID = 15;

        public static final Double LEFT_TICKS_PER_REV = 79675.000000 / Units.inchesToMeters(75); // 18600.0
        public static final Double RIGHT_TICKS_PER_REV = 84660.000000 / Units.inchesToMeters(81); // 9326 19500.0

        public static final Double MAX_VELOCITY = 3.6; // 1 ft 0.3048 * 5
        public static final Double MAX_ACCELERATION = 0.777; // 0.1 * 5
        public static final Double MAX_VOLTS = 100.0;

        public static final Double kTrackwidthMeters = 0.635; // 4.211691140585359 0.635
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final Double kS = 0.843; // 0.37
        public static final Double kV = 0.327; // 0.224
        public static final Double kA = 0.014; // 0.0109
        public static final Double kP = 0.000324; // 0.000439
        public static final Double kD = 0.000158; // 0.000183

        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                kS,
                kV,
                kA
            ),
            kDriveKinematics, 10
        );

        public static final TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION).setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    
        public static final double[] HEADING_PID = {0.014, 0.011, 0.0023}; // 0.014 0.011 0.0023 0.014 0.013 0.0033
        public static final double[] FAST_HEADING_PID = {0.2, 0.008, 0.1};

        public static final double TURN_TOLERANCE = 0.2; // 0.01
        public static final double TURN_RATE_TOLERANCE = 10.0;

        public static final double MAX_TURN_RATE = 100.0;
        public static final double MAX_TURN_ACCEL = 300.0;

        public static final double[] DIST_PID = {0.00009, 0.00003, 0.000001}; 

        public static final double DIST_TOLERANCE = 50.0;
        public static final double DIST_RATE_TOLERANCE = 300.0;

        public static final double MAX_DIST_VEL = 10.0;
        public static final double MAX_DIST_ACCEL = 30.0;

        public static final double[] STABILIZATION_PID = {0.3, 0.0, 0.003};
    }

    public static final class Turret {}

    public static final class Shooter {
        public static final Integer SHOOTER_1_ID = 2;
        public static final Integer SHOOTER_2_ID = 13;
        public static final Double SHOOTER_TICKS_PER_REV = 20000.0;

        public static final Double kP = 0.037; // 0.037
        public static final Double kI = 0.0;
        public static final Double kD = 0.0;
        public static final Double kF = 0.07; // 0.07 0.1 0.13
    }

    public static final class Intake {
        public static final Integer INTAKE_ID = 4;
        public static final Integer XFACTOR_ID = 1;
    }

    public static final class HorizIndexer {
        public static final Integer LEFT_INDEXER_ID = 5;
        public static final Integer RIGHT_INDEXER_ID = 10;
    }

    public static final class VertIndexer {
        public static final Integer INDEXER_ID = 11;
    }

    public static final class Climber {
        public static final Integer LEFT_CLIMBER_ID = 3;
        public static final Integer RIGHT_CLIMBER_ID = 12;
    }

    public static final class Playstation {
        
        // Driver Controls
        public static final Integer USBID = 0;

        // Axis
        public static final Integer LeftXAxis = 0;
        public static final Integer LeftYAxis = 1;
        public static final Integer RightXAxis = 2;
        public static final Integer RightYAxis = 5;

        // Trigger
        public static final Integer LeftTrigger = 3;
        public static final Integer RightTrigger = 4;

        // Bumper
        public static final Integer LeftBumper = 5;
        public static final Integer RightBumper = 6;

        // Buttons
        public static final Integer SquareButton = 1;
        public static final Integer XButton = 2;
        public static final Integer CircleButton = 3;
        public static final Integer TriangleButton = 4;

        public static final Integer LeftTriggerButton = 7;
        public static final Integer RightTriggerButton = 8;

        public static final Integer LeftButton = 9;
        public static final Integer RightButton = 10;

        public static final Integer LeftJoystickButton = 11;
        public static final Integer RightJoystickButton = 12;
        public static final Integer MiddleButton = 13;
        public static final Integer BigButton = 14;

        // POV Button
        public static final Integer NorthPOVButton = 0;
        public static final Integer NorthEastPOVButton = 45;
        public static final Integer EastPOVButton = 90;
        public static final Integer SouthEastPOVButton = 135;
        public static final Integer SouthPOVButton = 180;
        public static final Integer SouthWestPOVButton = 225;
        public static final Integer WestPOVButton = 270;
        public static final Integer NorthWestPOVButton = 315;
    }
}
