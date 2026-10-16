package frc.robot.state.standard;

import frc.robot.state.InputEvent;
import frc.robot.state.State;
import frc.robot.state.StateMachine;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployGoal;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;

public class Intake implements State {

  @Override
  public void enter() {
    rollerSubsystem.setGoal(RollerGoal.INTAKE);
    deploySubsystem.setGoal(DeployGoal.DEPLOY);
  }

  @Override
  public void exit() {} 

  @Override
  public State applyInput(InputEvent event) {
    switch (event) {
      case L2_PRESSED:
        return StateMachine.scoreOrPass();
      case L1_PRESSED:
        return new Neutral();
      default:
        return this;
    }
  }
}
