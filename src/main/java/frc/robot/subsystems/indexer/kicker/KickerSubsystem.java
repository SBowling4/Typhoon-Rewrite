package frc.robot.subsystems.indexer.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.InputEvent;
import frc.robot.state.StateMachine;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;

  private static final double JAM_DEBOUNCE_SECONDS = 0.1;
  private static final double UNJAM_SECONDS = 0.15;
  private static final double UNJAM_COOLDOWN_SECONDS = 0.75;

  private final Debouncer jamDebouncer = new Debouncer(JAM_DEBOUNCE_SECONDS, DebounceType.kBoth);
  private boolean unjamming = false;
  private double unjamStartTime = 0;
  private double lastUnjamTime = 0;

  private KickerGoal goal = KickerGoal.STOP;

  private double startSpinUpTime = 0;
  private boolean wasShooting = false;

  public KickerSubsystem() {
    this.kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.Feedback.SensorToMechanismRatio = 3;

    kickerMotor.getConfigurator().apply(cfg);
  }

  public void setGoal(KickerGoal goal) {
    this.goal = goal;
  }

  public Command kickCommand(KickerGoal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> stop(), this);
  }

  public boolean isJammed() {
    double currentTime = Timer.getFPGATimestamp();

    // Give time to spin up
    if (currentTime - startSpinUpTime < KickerConstants.SPINUP_TIME) return false;

    // Ignore states where we shouldn't be moving forward
    if (goal == KickerGoal.STOP || goal == KickerGoal.OUT) return false;

    // Only care when shooter is ready (same as spindexer)
    if (!StateMachine.getInstance().shooterReady()) return false;

    return kickerMotor.getStatorCurrent().getValueAsDouble() > KickerConstants.JAM_CURRENT.in(Amps);
  }

  /** Fires JAM_DETECTED / UNJAM_FINISHED into the state machine. Call once per loop. */
  private void updateUnjam() {
    double now = Timer.getFPGATimestamp();

    // Must run every loop so the debouncer's timing stays correct.
    boolean jammed = jamDebouncer.calculate(isJammed());

    if (unjamming) {
      if (now - unjamStartTime > UNJAM_SECONDS) {
        unjamming = false;
        StateMachine.getInstance().applyInput(InputEvent.UNJAM_FINISHED);
      }
    } else if (jammed && now - lastUnjamTime > UNJAM_COOLDOWN_SECONDS) {
      unjamming = true;
      unjamStartTime = now;
      lastUnjamTime = now;
      StateMachine.getInstance().applyInput(InputEvent.JAM_DETECTED);
    }

    DogLog.log("Spindexer/Unjam Active", unjamming);
  }

  public void stop() {
    kickerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Kicker/Goal", goal);
    DogLog.log("Kicker/Velocity", kickerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log("Kicker/Is Jammed", isJammed());
    DogLog.log("Kicker/Current", kickerMotor.getStatorCurrent().getValueAsDouble(), Amps);
  }

  @Override
  public void periodic() {
    log();
    updateUnjam();

    switch (goal) {
      case SHOOTING:
        if (StateMachine.getInstance().shooterReady()) {
          if (!wasShooting) {
            startSpinUpTime = Timer.getFPGATimestamp();
            wasShooting = true;
          }
          kickerMotor.set(KickerGoal.IN.getPower());
        } else {
          kickerMotor.stopMotor();
          wasShooting = false;
        }
        break;
      case STOP:
        kickerMotor.stopMotor();
        break;
      default:
        kickerMotor.set(goal.getPower());
        break;
    }
  }
}
