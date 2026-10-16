package frc.robot.state.override;

import frc.robot.state.InputEvent;
import frc.robot.state.State;

import java.util.Map;

public interface OverrideState extends State {
  Map<InputEvent, OverrideState> overrideInputs =
      Map.of(
          InputEvent.CROSS_PRESSED, new ReverseIntake(),
          InputEvent.JAM_DETECTED, new ReverseIntake());

  @Override
  default void exit() {}
}
