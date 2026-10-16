package frc.robot.state.standard;

import frc.robot.state.State;
import frc.robot.state.StateMachine;
import frc.robot.state.inputs.InputEvent;

public class Neutral implements State {

  @Override
  public void enter() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'enter'");
  }

  @Override
  public void exit() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'exit'");
  }

  @Override
  public State applyInput(InputEvent event) {
    switch (event) {
      case R2_PRESSED:
        return StateMachine.scoreOrPass();
      case L1_PRESSED:
        return new Intake();
      default:
        return this;
    }
  }
}
