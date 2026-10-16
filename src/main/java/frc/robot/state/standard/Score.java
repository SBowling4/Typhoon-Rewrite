package frc.robot.state.standard;

import frc.robot.RobotContainer;
import frc.robot.state.InputEvent;
import frc.robot.state.State;
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
  public void exit() {}

  @Override
  public State applyInput(InputEvent event) {
    switch (event) {
      case R2_RELEASED:
        return deploySubsystem.getGoal() == DeployGoal.DEPLOY
            ? new Intake()
            : new Neutral();
      default:
        return this;
    }
  }
}
