package frc.robot.state.override;

import frc.robot.state.InputEvent;
import frc.robot.state.State;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;

public class ReverseIntake implements OverrideState {
  @Override
  public void enter() {
    spindexerSubsystem.setGoal(SpindexerGoal.OUT);
    kickerSubsystem.setGoal(KickerGoal.OUT);
  }

  @Override
  public State applyInput(InputEvent event) {
    switch (event) {
      case CROSS_RELEASED:
      case UNJAM_FINISHED:
        return null;
      default:
        return this;
    }
  }
}
