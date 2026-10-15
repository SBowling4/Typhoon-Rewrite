package frc.robot.util.state.impl;

import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployState;
import frc.robot.util.state.InputEvent;
import frc.robot.util.state.State;

public class Score implements State {
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
            case R2_RELEASED:
                return RobotContainer.getDeploySubsystem().getState() == DeployState.DEPLOY ? new Intake() : new Neutral();        
            default:
                return this;
        }
    }
    
}
