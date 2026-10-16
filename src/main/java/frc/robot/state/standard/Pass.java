package frc.robot.state.standard;

import frc.robot.state.State;
import frc.robot.state.inputs.InputEvent;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelGoal;
import frc.robot.subsystems.shooter.hood.HoodConstants.HoodGoal;

public class Pass implements State {

  @Override
  public void enter() {
    flywheelSubsystem.setGoal(FlywheelGoal.PASSING);
    hoodSubsystem.setGoal(HoodGoal.PASSING);

    rollerSubsystem.setGoal(RollerGoal.INTAKE);

    spindexerSubsystem.setGoal(SpindexerGoal.SHOOTING);
    kickerSubsystem.setGoal(KickerGoal.SHOOTING);
  }

  @Override
  public void exit() {
    flywheelSubsystem.setGoal(FlywheelGoal.IDLE);
    hoodSubsystem.setGoal(HoodGoal.NEUTRAL);

    rollerSubsystem.setGoal(RollerGoal.STOP);

    spindexerSubsystem.setGoal(SpindexerGoal.STOP);
    kickerSubsystem.setGoal(KickerGoal.STOP);
  }

  @Override
  public State applyInput(InputEvent event) {
    switch (event) {
      case R2_RELEASED:
        return new Neutral();
      default:
        return this;
    }
  }
}
