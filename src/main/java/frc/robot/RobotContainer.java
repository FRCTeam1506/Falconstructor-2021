package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.Auto.Goal;
import frc.robot.Constants.Auto.Position;
import frc.robot.commands.Climber.Control;
import frc.robot.commands.Climber.Extend;
import frc.robot.commands.Climber.Retract;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.DriveStraight;
import frc.robot.commands.Drivetrain.FastRamsete;
import frc.robot.commands.Drivetrain.StandardRamsete;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.TurnToAngleProfiled;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.HorizIndexRevCycle;
import frc.robot.commands.HorizIndexer.StopHorizIndexer;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.ExtendAndOutake;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.commands.Intake.StopIntakeIntake;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.commands.Macros.TestMaster;
import frc.robot.commands.Macros.Unjam;
import frc.robot.commands.Macros.Shooter.First;
import frc.robot.commands.Macros.Shooter.Last;
import frc.robot.commands.Macros.Shooter.Second;
import frc.robot.commands.Macros.Shooter.Third;
import frc.robot.commands.Macros.Tests.TestMechanisms;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.commands.Shifter.SetToLowGear;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.VertIndexer.StopVertIndexer;
import frc.robot.commands.auton.LeftSimple;
import frc.robot.commands.auton.MiddleSimple;
import frc.robot.commands.auton.Nothing;
import frc.robot.commands.auton.Right6Ball;
import frc.robot.commands.auton.RightSimple;
import frc.robot.commands.defaults.HorizIndexer_Stop;
import frc.robot.commands.defaults.Intake_RetractAndStop;
import frc.robot.commands.defaults.Shifter_SetToHighGear;
import frc.robot.commands.defaults.Shooter_Stop;
import frc.robot.commands.defaults.VertIndexer_Stop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;
import frc.robot.utils.TrajectoryLoader;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  private final XboxController simulator = new XboxController(2);
  private final Joystick testinator = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  protected static final Drivetrain drivetrain = new Drivetrain();
  protected static final Shifter shifter = new Shifter();
  protected static final Shooter shooter = new Shooter();
  protected static final Intake intake = new Intake();
  protected static final HorizIndexer horizIndexer = new HorizIndexer();
  protected static final VertIndexer vertIndexer = new VertIndexer();
  protected static final Climber climber = new Climber();

  private SendableChooser<Constants.Auto.Position> positionChooser = new SendableChooser<>();
  private SendableChooser<Constants.Auto.Goal> goalChooser = new SendableChooser<>();

  protected final Command test = new TestMaster(drivetrain, shifter, intake, horizIndexer, vertIndexer, shooter);
  private final Command extendAndIntake = new ExtendAndIntake(intake);
  private final Command extendAndOutake = new ExtendAndOutake(intake);
  private final Command alignCommand = new Align(drivetrain);
  private final Command horizontalIndexCommand = new HorizIndex(horizIndexer);
  private final Command setToLowGearCommand = new SetToLowGear(shifter);
  private final Command unjamCommand = new Unjam(horizIndexer, vertIndexer, intake);
  private final Command indexAndShootCommand = new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter);
  private final Command extendCommand = new Extend(climber);
  private final Command retractCommand = new Retract(climber);
  private static final Command d_driveStraight = new DriveStraight(drivetrain);

  public static Trajectory Six_Ball_Right_1, Six_Ball_Right_2, Six_Ball_Right_3;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //? Configure the button bindings
    configureButtonBindings();

    //? Set Default Commands
    setDefaultCommands();

    //? Create Trajectories
    createTrajectories();

    //? Setup Auton Chooser
    setupAutonChooser();
  }

  private void configureButtonBindings() {
    //? Driver Controls
    new JoystickButton(driver, Constants.Playstation.TriangleButton).whenPressed(alignCommand.withTimeout(3.0));
    new JoystickButton(driver, Constants.Playstation.XButton).whileHeld(horizontalIndexCommand);
    new JoystickButton(driver, Constants.Playstation.LeftBumper).whileHeld(new SetToLowGear(shifter));
    new JoystickButton(driver, Constants.Playstation.RightBumper).whileHeld(extendAndIntake);

    //? Operator Controls
    new JoystickButton(operator, Constants.Playstation.LeftBumper).whileHeld(extendAndIntake);
    new JoystickButton(operator, Constants.Playstation.RightBumper).whileHeld(indexAndShootCommand);
    new JoystickButton(operator, Constants.Playstation.XButton).whileHeld(unjamCommand);
    new JoystickButton(operator, Constants.Playstation.CircleButton).whileHeld(extendAndOutake);
    new JoystickButton(operator, Constants.Playstation.TriangleButton).whileHeld(new HorizIndexRevCycle(horizIndexer));
    new JoystickButton(operator, Constants.Playstation.BigButton).whenPressed(extendCommand);
    new JoystickButton(operator, Constants.Playstation.MiddleButton).whenPressed(retractCommand);

    new POVButton(operator, Constants.Playstation.NorthPOVButton).whileHeld(new First(climber, drivetrain, intake, horizIndexer, vertIndexer, shooter));
    new POVButton(operator, Constants.Playstation.WestPOVButton).whileHeld(new Second(climber, intake, horizIndexer, vertIndexer, shooter));
    new POVButton(operator, Constants.Playstation.SouthPOVButton).whileHeld(new Third(climber, intake, horizIndexer, vertIndexer, shooter));
    new POVButton(operator, Constants.Playstation.EastPOVButton).whileHeld(new Last(climber, intake, horizIndexer, vertIndexer, shooter));

    //? Test Controller Controls
    // new JoystickButton(testinator, Constants.Playstation.BigButton).whileHeld(new TestMechanisms(intake, horizIndexer, vertIndexer, shooter));
  }


  private void setDefaultCommands() {

    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        shifter,
        () -> -driver.getRawAxis(Constants.Playstation.LeftYAxis),
        () -> driver.getRawAxis(Constants.Playstation.RightXAxis)
      )
    );

    // drivetrain.setDefaultCommand(
    //   new ArcadeDrive(
    //     drivetrain,
    //     shifter,
    //     () -> -simulator.getRawAxis(1),
    //     () -> simulator.getRawAxis(4)
    //   )
    // );

    shifter.setDefaultCommand(
      new Shifter_SetToHighGear(shifter)
    );

    shooter.setDefaultCommand(
      new Shooter_Stop(shooter)
    );

    intake.setDefaultCommand(
      new Intake_RetractAndStop(intake)
    );

    horizIndexer.setDefaultCommand(
      new HorizIndexer_Stop(horizIndexer)
    );

    vertIndexer.setDefaultCommand(
      new VertIndexer_Stop(vertIndexer)
    );

    climber.setDefaultCommand(
      new Control(
        climber, 
        () -> operator.getRawAxis(Constants.Playstation.LeftYAxis), 
        () -> operator.getRawAxis(Constants.Playstation.RightYAxis)
      )
    );

  }

  private void setupAutonChooser() {
    //? Position Auton Chooser
    positionChooser.setDefaultOption("Nothing", Position.Nothing);
    positionChooser.addOption("Left", Position.Left);
    positionChooser.addOption("Middle", Position.Middle);
    positionChooser.addOption("Right", Position.Right);
    Shuffleboard.getTab("Autonomous").add("Position", positionChooser);

    //? Goal Auton Chooser
    goalChooser.setDefaultOption("Safe", Goal.Safe);
    goalChooser.addOption("Ambitious", Goal.Ambitious);
    Shuffleboard.getTab("Autonomous").add("Goal", goalChooser);
  }

  private void createTrajectories() {
    // ? Six Ball (Right) Auton
    Six_Ball_Right_1  = TrajectoryLoader.loadTrajectoryFromFile("sb-r-1");
    Six_Ball_Right_2  = TrajectoryLoader.loadTrajectoryFromFile("sb-r-2");
    Six_Ball_Right_3  = TrajectoryLoader.loadTrajectoryFromFile("sb-r-3");

    // ? Six Ball (Left) Auton

  }

  // public Command getAutonomousCommand() {
  //   //? Reset Sensors
  //   // drivetrain.resetEncoders();
  //   // drivetrain.resetGyro();
  //   // String name = "work";
  //   // drivetrain.resetOdometry(TrajectoryLoader.loadTrajectoryFromFile(name).getInitialPose());
  //   // System.out.println(TrajectoryLoader.loadTrajectoryFromFile(name).getInitialPose());

  //   // return standardRamseteRevCommand(name);
  //   // return test6Ball2();
  //   return new RamseteTriggers(drivetrain, intake);

  //   // return new ParallelCommandGroup(
  //   //   new DefaultSetToHighGear(shifter),
  //   //   // test_fwd()
  //   //   // standardRamseteCommand("fwd")
  //   //   standardRamseteRevCommand("fwd")
  //     // new RamseteCommand(
  //     //   TrajectoryLoader.loadTrajectoryFromFile("u_curve_rev"),
  //     //   drivetrain::getPose,
  //     //   new RamseteController(),
  //     //   new SimpleMotorFeedforward(
  //     //     Constants.Drivetrain.kS,
  //     //     Constants.Drivetrain.kV,
  //     //     Constants.Drivetrain.kA
  //     //   ),
  //     //   Constants.Drivetrain.kDriveKinematics,
  //     //   drivetrain::getWheelSpeedsRev,
  //     //   new PIDController(1.1, 0.01, 0.15),
  //     //   new PIDController(1.5, 0.01, 0.05),
  //     //   drivetrain::tankDriveVoltsRev,
  //     //   drivetrain
  //     // )
  //   // );
  // }


  public Command getAutonomousCommand() {

    //? Reset Sensors
    drivetrain.resetEncoders();
    drivetrain.resetGyro();

    Position pos = positionChooser.getSelected();
    Goal goal = goalChooser.getSelected();

    if (pos == Position.Nothing) return new Nothing();
    
    else if (pos == Position.Left) { 
      if (goal == Goal.Safe) return new LeftSimple(drivetrain, intake, horizIndexer, vertIndexer, shooter);
      // else if (goal == Goal.Ambitious) return test5Ball();
    }

    else if (pos == Position.Middle) return new MiddleSimple(drivetrain, intake, horizIndexer, vertIndexer, shooter); 

    else if (pos == Position.Right) {
      if (goal == Goal.Safe) return new RightSimple(drivetrain, intake, horizIndexer, vertIndexer, shooter); 
      else if (goal == Goal.Ambitious) {
        return new Right6Ball(drivetrain, intake, horizIndexer, vertIndexer, shooter, Six_Ball_Right_1, Six_Ball_Right_2, Six_Ball_Right_3);
      }
    }

    else return new Nothing();

    return new InstantCommand(() -> drivetrain.tankDrive(0.0, 0.0));
  }

  // public Command test6Ball2() {                              // 0.67
  //   return new TankDrive(drivetrain, -0.87, -0.87).withTimeout(0.58).andThen(new Shoot(shooter, 20000.0).withTimeout(0.01)
  //     ).andThen(() -> drivetrain.setPipeline(0)
  //     ).andThen(
  //     new SequentialCommandGroup(
  //       new Align(drivetrain, 1.0).withTimeout(2.0),
  //       new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 0.5, 20000.0).withTimeout(3.0)
  //     ).andThen(
  //       new ParallelCommandGroup(
  //         new StopHorizIndexer(horizIndexer),
  //         new StopVertIndexer(vertIndexer),
  //         new StopShooter(shooter),
  //         new StopIntakeIntake(intake)
  //       ).withTimeout(0.01)
  //     ).andThen(
  //       () -> drivetrain.resetOdometry(Six_Ball_1.getInitialPose())
  //     ).andThen(
  //       new ParallelCommandGroup(
  //         new ExtendAndIntake(intake).withTimeout(5.0),
  //         new StandardRamsete(drivetrain, Six_Ball_1)
  //       ).withTimeout(7.0)
  //     ).andThen(new frc.robot.commands.Intake.Retract(intake).withTimeout(0.01)
  //     ).andThen(new Shoot(shooter, 20000.0).withTimeout(0.87)
  //     ).andThen(
  //       new TankDrive(drivetrain, -0.6, -0.6).withTimeout(0.56) // 0.66
  //     ).andThen(
  //       new TankDrive(drivetrain, 0.5, -0.5).withTimeout(0.68) // 0.675
  //     ).andThen(
  //       new TankDrive(drivetrain, 0.0, 0.0).withTimeout(0.01)
  //     ).andThen(() -> drivetrain.setPipeline(1)
  //     ).andThen(
  //       new SequentialCommandGroup(
  //         new Align(drivetrain, 1.0).withTimeout(2.0),
  //         new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 0.5, 20000.0).withTimeout(3.0)
  //       )
  //     )
  //   // ).andThen(
  //   //   new TurnToAngleProfiled(drivetrain, 180.0)
  //   // ).andThen(
  //   //   new ParallelCommandGroup(
  //   //     new IntakeIntake(intake),
  //   //     new DriveForward(drivetrain, 0.5).withTimeout(2.5)
  //   //   ).withTimeout(2.6)
  //   // ).andThen(
  //   //   new DriveBackward(drivetrain, 0.5).withTimeout(2.2)
  //   // ).andThen(
  //   //   new TurnToAngleProfiled(drivetrain, 180.0)
  //   // ).andThen(
  //   //   new ParallelCommandGroup(
  //   //     new Align(drivetrain).withTimeout(3.0),
  //   //     new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
  //   //   )
  //   ).andThen(() -> drivetrain.tankDrive(0.0, 0.0));
  // }

  // public Command test6Ball() {
  //   return new TankDrive(drivetrain, -0.5, -0.5).withTimeout(1.0).andThen(
  //     new ParallelCommandGroup(
  //       new Align(drivetrain, 1.0).withTimeout(3.0),
  //       new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 2.0, 20000.0)
  //     ).withTimeout(5.5)
  //     .andThen(
  //       new ParallelCommandGroup(
  //         new StopHorizIndexer(horizIndexer),
  //         new StopVertIndexer(vertIndexer),
  //         new StopShooter(shooter),
  //         new StopIntakeIntake(intake)
  //       ).withTimeout(0.01)
  //     ).andThen(
  //       () -> drivetrain.resetOdometry(Six_Ball_1.getInitialPose())
  //     ).andThen(
  //       new ParallelCommandGroup(
  //         new ExtendAndIntake(intake).withTimeout(5.0),
  //         new StandardRamsete(drivetrain, Six_Ball_1)
  //       ).withTimeout(7.0)
  //     ).andThen(new frc.robot.commands.Intake.Retract(intake).withTimeout(0.01)
  //     ).andThen(
  //         new TurnToAngleProfiled(drivetrain, -180)
  //     ).andThen(
  //       () -> drivetrain.resetOdometry(Six_Ball_2.getInitialPose()))
  //     ).andThen(
  //       new FastRamsete(drivetrain, Six_Ball_2)
  //     ).andThen(
  //       new ParallelCommandGroup(
  //         new Align(drivetrain, 1.0).withTimeout(3.0),
  //         new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter, 2.0, 20000.0)
  //       ).withTimeout(5.5)
  //   // ).andThen(
  //   //   new TurnToAngleProfiled(drivetrain, 180.0)
  //   // ).andThen(
  //   //   new ParallelCommandGroup(
  //   //     new IntakeIntake(intake),
  //   //     new DriveForward(drivetrain, 0.5).withTimeout(2.5)
  //   //   ).withTimeout(2.6)
  //   // ).andThen(
  //   //   new DriveBackward(drivetrain, 0.5).withTimeout(2.2)
  //   // ).andThen(
  //   //   new TurnToAngleProfiled(drivetrain, 180.0)
  //   // ).andThen(
  //   //   new ParallelCommandGroup(
  //   //     new Align(drivetrain).withTimeout(3.0),
  //   //     new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
  //   //   )
  //   ).andThen(() -> drivetrain.tankDrive(0.0, 0.0));
  // }

  public static Command getDriveStraight() {
    return d_driveStraight;
  }
}
