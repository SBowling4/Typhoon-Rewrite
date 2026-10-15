package frc.robot.util.state;

public interface State {
    void enter();
    void exit();

    public State applyInput(InputEvent event);
}
