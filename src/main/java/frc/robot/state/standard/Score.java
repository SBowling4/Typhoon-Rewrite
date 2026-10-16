package frc.robot.state.standard;

import frc.robot.RobotContainer;
import frc.robot.state.State;
import frc.robot.state.inputs.InputEvent;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployGoal;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelGoal;
import frc.robot.subsystems.shooter.hood.HoodConstants.HoodGoal;

public class Score implements State {
  @Override
  public void enter() {
    flywheelSubsystem.setGoal(FlywheelGoal.SCORING);
    hoodSubsystem.setGoal(HoodGoal.SCORING);

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
        return RobotContainer.getDeploySubsystem().getState() == DeployGoal.DEPLOY
            ? new Intake()
            : new Neutral();
      default:
        return this;
    }
  }
}
