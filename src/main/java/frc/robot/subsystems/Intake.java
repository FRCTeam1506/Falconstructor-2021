package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intake = new TalonFX(Constants.Intake.INTAKE_ID);
    private final Solenoid xfactor = new Solenoid(Constants.Intake.XFACTOR_ID);

    public Intake() {
        this.intake.configFactoryDefault();
        this.intake.setInverted(false);

        dashboard();
    }

    public void intakeFwd() {
        this.intake.set(ControlMode.PercentOutput, 0.77); // 0.55
    }

    public void intakeFwd(double pwr) {
        this.intake.set(ControlMode.PercentOutput, pwr);
    }

    public void intakeRev() {
        this.intake.set(ControlMode.PercentOutput, -0.33);
    }

    public void intakeRev(double pwr) {
        this.intake.set(ControlMode.PercentOutput, -pwr);
    }

    public void stopIntake() {
        this.intake.set(ControlMode.PercentOutput, 0.0);
    }

    public void retract() {
        if(getState() != "Retracted") this.xfactor.set(false);
    }

    public void extend() {
        if(getState() != "Extended") this.xfactor.set(true);
    }

    public String getState() {
        return this.xfactor.get() ? "Extended" : "Retracted";
    }

    private void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        tab.addBoolean("State", () -> this.xfactor.get()).withWidget(BuiltInWidgets.kBooleanBox);
    }

    @Override
    public void periodic() {}
}