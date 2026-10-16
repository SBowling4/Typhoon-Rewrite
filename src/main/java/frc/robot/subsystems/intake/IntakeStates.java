package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.deploy.DeployConstants;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployGoal;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;

// I'm like 90% sure this doesn't do anthing but I'm scared to delete it so here we are
public enum IntakeStates {
  INTAKE(RollerConstants.RollerGoal.INTAKE, DeployConstants.DeployGoal.DEPLOY),
  OUT(RollerConstants.RollerGoal.OUT, DeployConstants.DeployGoal.DEPLOY),
  STOP(RollerConstants.RollerGoal.STOP, DeployConstants.DeployGoal.RETRACT),
  AGITATE(RollerConstants.RollerGoal.STOP, DeployConstants.DeployGoal.AGITATE);

  private RollerGoal rollerState;
  private DeployGoal deployState;

  private IntakeStates(RollerGoal rollerState, DeployGoal deployState) {
    this.rollerState = rollerState;
    this.deployState = deployState;
  }

  public DeployGoal getDeployState() {
    return this.deployState;
  }

  public RollerGoal getRollerState() {
    return this.rollerState;
  }
}
