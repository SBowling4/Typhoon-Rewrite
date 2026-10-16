package frc.robot.auto.commands.left;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.state.StateMachine;
import frc.robot.state.standard.Intake;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerGoal;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerGoal;

public class LeftDoublePickup extends BaseAuto {
  public LeftDoublePickup() {
    super(
        Paths.leftShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToPickup),
            Commands.waitTime(Seconds.of(.5))
                .andThen(StateMachine.getInstance().setStateCommand(new Intake()))),
        Commands.race(
            AutoBuilder.followPath(Paths.leftPickupToShoot),
            RobotContainer.getKickerSubsystem().kickCommand(KickerGoal.OUT),
            RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerGoal.OUT)),
        shootSequence(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToLowPickup),
            Commands.parallel(
                    RobotContainer.getKickerSubsystem().kickCommand(KickerGoal.OUT),
                    RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerGoal.OUT))
                .withTimeout(Seconds.of(1))
                .andThen(StateMachine.getInstance().setStateCommand(new Intake()))),
        AutoBuilder.followPath(Paths.leftLowPickupToShoot),
        shootSequence(Seconds.of(10)));
  }
}
