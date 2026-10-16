package frc.robot.state;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.state.inputs.InputEvent;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;

/**
 * Watches the indexer for jams and fires JAM_DETECTED / UNJAM_FINISHED into the state machine. The
 * Unjam override does the actual work. Call periodic() once per loop (e.g. from
 * Robot.robotPeriodic()).
 */
public class Unjammer {

  private static final double DEBOUNCE_SECONDS = 0.1;
  private static final double UNJAM_SECONDS = 0.15;
  private static final double COOLDOWN_SECONDS = 0.75;

  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;

  private final Debouncer jamDebouncer = new Debouncer(DEBOUNCE_SECONDS, DebounceType.kBoth);

  private boolean unjamming = false;
  private double unjamStartTime = 0;
  private double lastUnjamTime = 0;

  public Unjammer(SpindexerSubsystem spindexer, KickerSubsystem kicker) {
    this.spindexer = spindexer;
    this.kicker = kicker;
  }

  public void periodic() {
    double now = Timer.getFPGATimestamp();

    // Must be called every loop so the debouncer's timing stays correct.
    boolean jammed = jamDebouncer.calculate(spindexer.isJammed() || kicker.isJammed());

    StateMachine stateMachine = StateMachine.getInstance();

    if (unjamming) {
      if (now - unjamStartTime > UNJAM_SECONDS) {
        unjamming = false;
        stateMachine.applyInput(InputEvent.UNJAM_FINISHED);
      }
    } else if (jammed && now - lastUnjamTime > COOLDOWN_SECONDS) {
      unjamming = true;
      unjamStartTime = now;
      lastUnjamTime = now;
      stateMachine.applyInput(InputEvent.JAM_DETECTED);
    }

    DogLog.log("Unjammer/Indexer Jammed", jammed);
    DogLog.log("Unjammer/Unjam Active", unjamming);
  }
}
