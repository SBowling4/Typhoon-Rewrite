package frc.robot.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.state.inputs.InputEvent;
import frc.robot.state.override.OverrideState;
import frc.robot.state.standard.Pass;
import frc.robot.state.standard.Score;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.util.field.Region;
import java.util.Optional;

public class StateMachine {

  private State currentState;
  private boolean overrideActive = false;

  private boolean flywheelSpunUp = false;

  private static StateMachine instance;

  public void applyInput(InputEvent event) {
    if (OverrideState.overrideInputs.containsKey(event) || overrideActive) {
      overrideActive = true;
      OverrideState overrideState = OverrideState.overrideInputs.get(event);
      overrideState.enter();

      if (overrideState.applyInput(event) != null) {
        return;
      } else {
        overrideActive = false;
      }
    }

    State nextState = currentState.applyInput(event);

    if (nextState != currentState) {
      flywheelSpunUp = false;
      currentState.exit();
      nextState.enter();
      currentState = nextState;
    }
  }

  public Command applyInputCommand(InputEvent event) {
    return Commands.runOnce(() -> applyInput(event));
  }

  public void setState(State state) {
    currentState.exit();
    currentState = state;
    currentState.enter();
  }

  public Command setStateCommand(State state) {
    return Commands.runOnce(() -> setState(state));
  }

  public boolean shooterReady() {
    ShotParameters params = ShotCalculator.getInstance().calculateParameters();

    if (RobotContainer.getFlywheelSubsystem().atTargetVelocity() && !flywheelSpunUp) {
      flywheelSpunUp = true;
    }

    return flywheelSpunUp
        && RobotContainer.getTurretSubsystem().atTargetAngle()
        && params.isValid();
  }

  public static StateMachine getInstance() {
    if (instance == null) {
      instance = new StateMachine();
    }
    return instance;
  }

  public static State scoreOrPass() {
    Pose2d pose = RobotContainer.getDrivetrain().getPose();
    Optional<Region> region = Region.getRegion(pose);
    if (region.isPresent()) {
      return region.get() == Region.UPPER_NEUTRAL_ZONE || region.get() == Region.LOWER_NEUTRAL_ZONE
          ? new Pass()
          : new Score();
    }
    return new Score();
  }
}
