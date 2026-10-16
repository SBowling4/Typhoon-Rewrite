package frc.robot.subsystems.intake.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.StateMachine;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerGoal;

public class RollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  private SparkClosedLoopController rollerController;

  private RollerGoal goal = RollerGoal.STOP;

  public RollerSubsystem() {
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.inverted(false);

    rollerMotorConfig
        .closedLoop
        .p(RollerConstants.kP)
        .feedForward
        .kV(RollerConstants.kV)
        .kS(RollerConstants.kS);

    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerController = rollerMotor.getClosedLoopController();

    goal = RollerConstants.RollerGoal.STOP;
  }

  public void setGoal(RollerGoal goal) {
    this.goal = goal;
  }

  public Command rollerCommand(RollerGoal goal) {
    return Commands.run(() -> setGoal(goal), this).finallyDo(() -> stop());
  }

  public void stop() {
    rollerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Intake/Roller/Goal", goal.name());
    DogLog.log("Intake/Roller/Target Velocity", goal.getVelocity());

    DogLog.log("Intake/Roller/Velocity", rollerMotor.getEncoder().getVelocity());
    DogLog.log("Intake/Roller/Voltage", rollerMotor.getBusVoltage());
  }

  @Override
  public void periodic() {
    log();

    switch (goal) {
      case STOP:
        rollerMotor.stopMotor();
        break;
      case SHOOTING:
        if (StateMachine.getInstance().shooterReady()) {
          rollerController.setSetpoint(RollerGoal.INTAKE.getVelocity(), ControlType.kVelocity);
        } else {
          rollerMotor.stopMotor();
        }
        break;
      default:
        rollerController.setSetpoint(goal.getVelocity(), ControlType.kVelocity);
        break;
    }
  }
}
