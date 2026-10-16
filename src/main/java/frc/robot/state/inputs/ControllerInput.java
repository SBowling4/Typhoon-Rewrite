package frc.robot.state.inputs;

import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Function;

/**
 * Every physical input on the PS5 controller, along with how to build a Trigger for it.
 *
 * <p>Each constant X must have matching InputEvent constants X_PRESSED and X_RELEASED.
 * ControllerBindings.bindAll() verifies this at startup and throws if one is missing.
 */
public enum ControllerInput {
  // Face buttons
  CROSS(CommandPS5Controller::cross),
  CIRCLE(CommandPS5Controller::circle),
  SQUARE(CommandPS5Controller::square),
  TRIANGLE(CommandPS5Controller::triangle),

  // D-pad
  POV_UP(CommandPS5Controller::povUp),
  POV_RIGHT(CommandPS5Controller::povRight),
  POV_DOWN(CommandPS5Controller::povDown),
  POV_LEFT(CommandPS5Controller::povLeft),

  // Bumpers
  L1(CommandPS5Controller::L1),
  R1(CommandPS5Controller::R1),

  // Triggers (digital button)
  L2(CommandPS5Controller::L2),
  R2(CommandPS5Controller::R2),

  // Stick buttons
  L3(CommandPS5Controller::L3),
  R3(CommandPS5Controller::R3),

  // Center / system buttons
  CREATE(CommandPS5Controller::create),
  OPTIONS(CommandPS5Controller::options),
  PS(CommandPS5Controller::PS),
  TOUCHPAD(CommandPS5Controller::touchpad),

  // Stick directions (digital threshold events). Pushing a stick up gives a NEGATIVE Y value.
  LEFT_STICK_UP(c -> c.axisLessThan(Axis.kLeftY.value, -Threshold.STICK)),
  LEFT_STICK_DOWN(c -> c.axisGreaterThan(Axis.kLeftY.value, Threshold.STICK)),
  LEFT_STICK_LEFT(c -> c.axisLessThan(Axis.kLeftX.value, -Threshold.STICK)),
  LEFT_STICK_RIGHT(c -> c.axisGreaterThan(Axis.kLeftX.value, Threshold.STICK)),

  RIGHT_STICK_UP(c -> c.axisLessThan(Axis.kRightY.value, -Threshold.STICK)),
  RIGHT_STICK_DOWN(c -> c.axisGreaterThan(Axis.kRightY.value, Threshold.STICK)),
  RIGHT_STICK_LEFT(c -> c.axisLessThan(Axis.kRightX.value, -Threshold.STICK)),
  RIGHT_STICK_RIGHT(c -> c.axisGreaterThan(Axis.kRightX.value, Threshold.STICK));

  /**
   * Lives in a nested class because enum constants are initialized before the enum's own static
   * fields, so the constants above can't reference a field declared here directly.
   */
  private static final class Threshold {
    /** How far a stick must be pushed (0 to 1) to count as "pressed". */
    static final double STICK = 0.5;
  }

  private final Function<CommandPS5Controller, Trigger> triggerFactory;

  ControllerInput(Function<CommandPS5Controller, Trigger> triggerFactory) {
    this.triggerFactory = triggerFactory;
  }

  public Trigger trigger(CommandPS5Controller controller) {
    return triggerFactory.apply(controller);
  }

  public InputEvent pressedEvent() {
    return InputEvent.valueOf(name() + "_PRESSED");
  }

  public InputEvent releasedEvent() {
    return InputEvent.valueOf(name() + "_RELEASED");
  }
}
