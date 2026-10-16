package frc.robot.state.inputs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.state.StateMachine;

public final class ControllerBindings {

  private ControllerBindings() {}

  /**
   * Routes every input on the controller into the state machine as a PRESSED / RELEASED event. Call
   * once from RobotContainer.configureBindings().
   */
  public static void bindAll(CommandPS5Controller controller) {
    StateMachine stateMachine = StateMachine.getInstance();

    for (ControllerInput input : ControllerInput.values()) {
      // valueOf throws immediately if an InputEvent is missing, so a typo fails at startup
      InputEvent pressed = input.pressedEvent();
      InputEvent released = input.releasedEvent();

      Trigger trigger = input.trigger(controller);
      trigger.onTrue(
          Commands.runOnce(() -> stateMachine.applyInput(pressed)).ignoringDisable(true));
      trigger.onFalse(
          Commands.runOnce(() -> stateMachine.applyInput(released)).ignoringDisable(true));
    }
  }
}
