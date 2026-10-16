package frc.robot.subsystems.indexer.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.InputEvent;
import frc.robot.state.StateMachine;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;
  private SpindexerGoal goal = SpindexerGoal.STOP;

  private static final double JAM_DEBOUNCE_SECONDS = 0.1;
  private static final double UNJAM_SECONDS = 0.15;
  private static final double UNJAM_COOLDOWN_SECONDS = 0.75;

  private final Debouncer jamDebouncer = new Debouncer(JAM_DEBOUNCE_SECONDS, DebounceType.kBoth);
  private boolean unjamming = false;
  private double unjamStartTime = 0;
  private double lastUnjamTime = 0;

  private double startSpinUpTime;

  private boolean wasShooting = false;

  public SpindexerSubsystem() {
    spinMotor = new SparkMax(SpindexerConstants.SPIN_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);
    spinMotorConfig.inverted(true);

    ClosedLoopConfig clCfg = new ClosedLoopConfig();
    clCfg.p(SpindexerConstants.kP).feedForward.kV(SpindexerConstants.kV).kS(SpindexerConstants.kS);

    spinMotorConfig.apply(clCfg);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    goal = SpindexerGoal.STOP;
  }

  public void setGoal(SpindexerGoal goal) {
    this.goal = goal;
  }

  public Command spinCommand(SpindexerGoal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> stop(), this);
  }

  public boolean isJammed() {
    if (Timer.getFPGATimestamp() - startSpinUpTime < SpindexerConstants.SPINUP_TIME) return false;
    if (goal == SpindexerGoal.STOP || goal == SpindexerGoal.OUT) return false;
    if (!StateMachine.getInstance().shooterReady()) return false;

    return spinMotor.getOutputCurrent() > 38.0;
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
    spinMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Spindexer/Goal", goal);
    DogLog.log("Spindexer/Is Jammed", isJammed());
    DogLog.log("Spindexer/Current", spinMotor.getOutputCurrent());
    DogLog.log("Spindexer/Velocity", spinMotor.getEncoder().getVelocity());
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
          spinMotor.set(goal.getPower());
        } else {
          stop();
          wasShooting = false;
        }
        break;

      case STOP:
        spinMotor.stopMotor();
        break;
      default:
        spinMotor.set(goal.getPower());
        break;
    }
  }
}
