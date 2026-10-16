package frc.robot.state;

import frc.robot.RobotContainer;
import frc.robot.state.inputs.InputEvent;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public interface State {
  CommandSwerveDrivetrain drivetrain = RobotContainer.getDrivetrain();

  FlywheelSubsystem flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
  HoodSubsystem hoodSubsystem = RobotContainer.getHoodSubsystem();
  TurretSubsystem turretSubsystem = RobotContainer.getTurretSubsystem();

  DeploySubsystem deploySubsystem = RobotContainer.getDeploySubsystem();
  RollerSubsystem rollerSubsystem = RobotContainer.getRollerSubsystem();

  SpindexerSubsystem spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
  KickerSubsystem kickerSubsystem = RobotContainer.getKickerSubsystem();

  void enter();

  void exit();

  public State applyInput(InputEvent event);
}
