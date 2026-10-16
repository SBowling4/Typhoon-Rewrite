package frc.robot.state.standard;

import frc.robot.state.InputEvent;
import frc.robot.state.State;
import frc.robot.state.StateMachine;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployGoal;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelGoal;
import frc.robot.subsystems.shooter.hood.HoodConstants.HoodGoal;

public class Neutral implements State {

  @Override
  public void enter() {
    flywheelSubsystem.setGoal(FlywheelGoal.IDLE);
    hoodSubsystem.setGoal(HoodGoal.NEUTRAL);

    rollerSubsystem.setGoal(RollerGoal.STOP);
    deploySubsystem.setGoal(DeployGoal.RETRACT);

    spindexerSubsystem.setGoal(SpindexerGoal.STOP);
    kickerSubsystem.setGoal(KickerGoal.STOP);
  }

  @Override
  public void exit() {}

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
