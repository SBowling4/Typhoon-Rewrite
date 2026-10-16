package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.hood.HoodConstants.HoodGoal;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor;

  private Rotation2d position;

  private HoodGoal goal = HoodGoal.NEUTRAL;

  public HoodSubsystem() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO_MOTOR;

    cfg.Slot0.kP = HoodConstants.kP;

    cfg.Slot0.kS = HoodConstants.kS;

    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.setPosition(Rotations.of(0));
  }

  public void setPosition(Rotation2d position) {
    hoodMotor.setControl(new PositionVoltage(position.getRotations()));
  }

  public Command setPositionCommand(Rotation2d position) {
    return Commands.startEnd(() -> setPosition(position), () -> stop(), this);
  }

  public Rotation2d getMotorPosition() {
    if (position != null) {
      return position;
    }

    position = Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    return position;
  }

  public Command applyVoltageCommand(double volts) {
    return Commands.run(() -> hoodMotor.setVoltage(volts), this).finallyDo(() -> stop());
  }

  public void setGoal(HoodGoal goal) {
    this.goal = goal;
  }

  public void log() {
    DogLog.log("Hood/Goal", goal);
    DogLog.log("Hood/Position", getMotorPosition().getDegrees(), Degrees);
    DogLog.log("Hood/Volts", hoodMotor.getMotorVoltage().getValueAsDouble(), Volts);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  private double startTime = 0;
  private boolean zeroing = false;

  // thank you scream
  public Command zero() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              startTime = Timer.getFPGATimestamp();
              zeroing = true;
            }),
        applyVoltageCommand(-1.0)
            .withDeadline(
                new WaitUntilCommand(() -> ((Timer.getFPGATimestamp() - startTime) > 1.0))),
        new InstantCommand(
            () -> {
              hoodMotor.setPosition(Rotations.of(0));
              zeroing = false;
            }));
  }

  @Override
  public void periodic() {
    log();

    if (zeroing) return;

    switch (goal) {
      case PASSING:
        ShotParameters passingParams = ShotCalculator.getInstance().calculateParameters();

        if (!passingParams.isValid()) {
          idle();
        } else {
          setPosition(passingParams.hoodAngle());
        }
        break;
      case SCORING:
        ShotParameters scoringParams = ShotCalculator.getInstance().calculateParameters();

        if (!scoringParams.isValid()) {
          idle();
        } else {
          setPosition(scoringParams.hoodAngle());
        }
        break;
      case NEUTRAL:
        setPosition(Rotation2d.kZero);
        break;
      default:
        break;
    }
  }

  public void clearCache() {
    position = null;
  }

  private Rotation2d simPos = Rotation2d.kZero;

  public void setSimPosition(Rotation2d pos) {
    this.simPos = pos;
  }

  public Rotation2d getSimPos() {
    return this.simPos;
  }

  @Override
  public void simulationPeriodic() {
    ShotParameters params = ShotCalculator.getInstance().calculateParameters();

    setSimPosition(params.hoodAngle());
  }
}
