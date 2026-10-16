package frc.robot.auto.commands.left;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.state.StateMachine;
import frc.robot.state.standard.Intake;

public class LeftSinglePickup extends BaseAuto {

  public LeftSinglePickup() {
    super(
        Paths.leftShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToPickup),
            Commands.waitTime(Seconds.of(1))
                .andThen(StateMachine.getInstance().setStateCommand(new Intake()))),
        AutoBuilder.followPath(Paths.rightPickupToShoot),
        shootSequence(20));
  }
}
