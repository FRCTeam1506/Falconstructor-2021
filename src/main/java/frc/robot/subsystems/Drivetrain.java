package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

    //? Drives
    private final TalonFX leftDriveMaster = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_MASTER_ID);
    private final TalonFX leftDrive = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_ID);
    private final TalonFX rightDriveMaster = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_MASTER_ID);
    private final TalonFX rightDrive = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_ID);

    private TalonFXInvertType talonFXInvertType = TalonFXInvertType.FollowMaster;

    private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 100.0, 120.0, 0);

    //? Limelight
    private boolean aligned, isRefreshed, targetFound;
    private Double x, y, area, targetDistance, previousX, previousY, previousDist;
    private Integer pipeline;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private PIDController leftDrivePID, rightDrivePID;

    public enum Piplelines {
        NearTargeting,
        FarTargeting,
        Red,
        Green,
        Blue,
        Yellow
    }

    //? Gyro
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private double targetHeading;

    //? Trajectory
    private DifferentialDriveOdometry odometry;

    //? Simulation
    private Field2d field = new Field2d();

    public Drivetrain() {
        this.leftDrive.configFactoryDefault();
        this.leftDriveMaster.configFactoryDefault();
        this.rightDrive.configFactoryDefault();
        this.rightDriveMaster.configFactoryDefault();

        // Set followers
        this.leftDrive.follow(this.leftDriveMaster);
        this.rightDrive.follow(this.rightDriveMaster);

        this.leftDriveMaster.setInverted(false);
        this.rightDriveMaster.setInverted(true);
        this.leftDrive.setInverted(talonFXInvertType);
        this.rightDrive.setInverted(talonFXInvertType);

        this.leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        this.rightDriveMaster.setNeutralMode(NeutralMode.Brake);

        this.leftDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        this.leftDrive.setNeutralMode(NeutralMode.Brake);
        this.rightDrive.setNeutralMode(NeutralMode.Brake);

        this.leftDrive.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightDrive.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        this.targetDistance = 5000.0;

        resetEncoders();

        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        this.x = 0.0;
        this.y = 0.0;

        dashboard();
    }

    //? Drive
    public void fwd(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, pwr);
    }

    public void rot(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, -pwr);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, leftSpeed);
        this.rightDriveMaster.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(leftVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
        this.rightDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(rightVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
    }

    public void fastTankDriveVolts(double leftVolts, double rightVolts) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(leftVolts/12.0 * 1.7, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
        this.rightDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(rightVolts/12.0 * 1.7, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
    }

    public void tankDriveVoltsRev(double leftVolts, double rightVolts) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(-leftVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
        this.rightDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(-rightVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
    }

    public void limitedArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 1.0, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 1.0, 3));
    }

    public void arcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 1.0, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 1.0, 3));
    }

    public void lowGearArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 1.2, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 1.2, 3));
    }

    public void highGearArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 0.7, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 0.7, 3));
    }

    public void velocityArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.Velocity, ((fwd + rot) * 0.8) * 500000);
        this.rightDriveMaster.set(ControlMode.Velocity, ((fwd - rot) * 0.8) * 500000);
    }

    public void regArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, (fwd + rot));
        this.rightDriveMaster.set(ControlMode.PercentOutput, (fwd - rot));
    }

    public void stop() {
        this.leftDriveMaster.set(ControlMode.PercentOutput, 0.0);
        this.rightDriveMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public double getRawLeftDistance() {
        return this.leftDriveMaster.getSelectedSensorPosition();
    }

    public double getRawRightDistance() {
        return this.rightDriveMaster.getSelectedSensorPosition();
    }

    public void resetEncoders() {
        this.leftDriveMaster.setSelectedSensorPosition(0, 0, 0);
        this.rightDriveMaster.setSelectedSensorPosition(0, 0, 0);
    }

    //? Limelight
    public void setPipeline(int index) {
        this.table.getEntry("pipeline").setNumber(index);
    }

    public void setPipeline(Piplelines pipe) {
        switch (pipe) {
            case NearTargeting:
                this.setPipeline(5);
                break;

            case FarTargeting:
                this.setPipeline(6);
                break;

            case Red:
                this.setPipeline(0);
                break;

            case Green:
                this.setPipeline(1);
                break;

            case Blue:
                this.setPipeline(2);
                break;

            case Yellow:
                this.setPipeline(3);
                break;
        
            default:
                this.setPipeline(5);
                break;
        }
    }

    public void setTargetDistance(Double distance) {
        this.targetDistance = distance;
    }

    public Double getX() {
        if(this.x != null) {
            return this.x;
        } else {
            return 0.0;
        }
    }

    public Double getY() {
        if(this.y != null) {
            return this.y;
        } else {
            return 0.0;
        }
    }

    public Double getArea() {
        if(this.area != null) {
            return this.area;
        } else {
            return 0.0;
        }
    }

    public int getPipeline() {
        return this.pipeline;
    }

    public boolean isRefreshed() {
        return this.isRefreshed;
    }

    public Double getDistance() {
        double val;
        // inches
        Double h2 = 96.0;
        Double h1 = 30.0;
        Double a1 = 1.0; // 0.258
        Double a2 = this.y;
        // return (h2 - h1) / Math.tan(a1 + a2);
        double dist = (h2 - h1) / (Math.tan((a1 + a2) * (Math.PI / 180)));
        if(isTargetFound()) {
            if(dist > 0) {
                val = (double) Math.ceil((dist / 1000.0)) * 1000.0;
                if(Math.abs(val) < 1) this.previousDist = val;
            } else {
                if(this.previousDist != null){
                    val = this.previousDist;
                } else {
                    val = 0;
                }
            }
            return val;
        } else {
            return 0.0;
        }
    }

    public Double getTargetDistance() {
        return this.targetDistance;
    }

    public boolean isTargetFound() {
        return this.targetFound;
    }

    public boolean isAligned() {
        return this.aligned;
    }

    public Double calculateDistance() {
        return getDistance() * 10.0;
    }

    //? Gyro
    public void resetGyro() {
    	navx.zeroYaw();
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double heading) {
        this.targetHeading = heading;
    }

    //? Trajectory
    public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
        leftDrivePID.setSetpoint(left.position);
        rightDrivePID.setSetpoint(right.position);
        this.leftDriveMaster.set(ControlMode.Position, leftDrivePID.getSetpoint());
        this.rightDriveMaster.set(ControlMode.Position, rightDrivePID.getSetpoint());
    }

    public Double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getTurnRate() {
        return navx.getRate();
    }

    public double getXCoordinate() {
        return this.odometry.getPoseMeters().getTranslation().getX();
    }

    public double getYCoordinate() {
        return this.odometry.getPoseMeters().getTranslation().getY();
    }

    public double getLeftDistanceMeters() {
        return leftDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.LEFT_TICKS_PER_REV;
    }

    public double getRightDistanceMeters() {
        return rightDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.RIGHT_TICKS_PER_REV;
    }

    public double getLeftVelocityMetersPerSecond() {
        // return leftDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.LEFT_TICKS_PER_REV);
        return leftDriveMaster.getSelectedSensorVelocity() / Constants.Drivetrain.LEFT_TICKS_PER_REV;
    }

    public double getRightVelocityMetersPerSecond() {
        // return rightDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.RIGHT_TICKS_PER_REV);
        return rightDriveMaster.getSelectedSensorVelocity() / Constants.Drivetrain.RIGHT_TICKS_PER_REV;
    }

    public double getAverageEncoderDistanceMeters() {
        return ( ( getLeftDistanceMeters() + getRightDistanceMeters() ) / 2.0 );
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(), 
            getRightVelocityMetersPerSecond()
        );
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetAll() {
        resetEncoders();
        resetGyro();
    }

    public void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.addNumber("Target-Distance", this::getTargetDistance).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Distance", this::getDistance).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Heading", this::getHeading).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Left-Dist-Meters", this::getLeftDistanceMeters).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Right-Dist-Meters", this::getRightDistanceMeters).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Left-Ticks", () -> this.leftDriveMaster.getSelectedSensorPosition()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Right-Ticks", () -> this.rightDriveMaster.getSelectedSensorPosition()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("X Coord", this::getXCoordinate).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Y Coord", this::getYCoordinate).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Left-Drive-Master-Temp", () -> this.leftDriveMaster.getTemperature()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Left-Drive-Temp", () -> this.leftDrive.getTemperature()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Right-Drive-Master-Temp", () -> this.rightDriveMaster.getTemperature()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Right-Drive-Temp", () -> this.rightDrive.getTemperature()).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Max-Temp", () -> Math.max(Math.max(this.leftDriveMaster.getTemperature(), this.leftDrive.getTemperature()), Math.max(this.rightDriveMaster.getTemperature(), this.rightDrive.getTemperature()))).withWidget(BuiltInWidgets.kGraph);

        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        limelightTab.addNumber("X Error", this::getX);
        limelightTab.addNumber("Y Error", this::getY);
        limelightTab.addNumber("Area", this::getArea);

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        
        odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            getLeftDistanceMeters(), 
            getRightDistanceMeters()
        );

        field.setRobotPose(odometry.getPoseMeters());

        if(!(CommandScheduler.getInstance().isScheduled(RobotContainer.getDriveStraight()))) {
            // System.out.println("Running");
            this.targetHeading = getHeading();
        }
        // this.targetHeading = getHeading();

        this.x = table.getEntry("tx").getDouble(0.0);
        this.y = table.getEntry("ty").getDouble(0.0);
        this.area = table.getEntry("ta").getDouble(0.0);
        this.targetFound = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        this.pipeline = table.getEntry("pipeline").getNumber(0).intValue();
        this.aligned = Math.abs(x) < 0.1 ? true : false;
        this.isRefreshed = this.x != previousX || this.y != previousY ? true : false;
        this.previousX = this.x;
        this.previousY = this.y;
        
        // this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    }
}