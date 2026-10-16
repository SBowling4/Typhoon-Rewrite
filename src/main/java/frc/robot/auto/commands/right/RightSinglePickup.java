package frc.robot.auto.commands.right;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.state.StateMachine;
import frc.robot.state.standard.Intake;
import frc.robot.state.standard.Score;

public class RightSinglePickup extends BaseAuto {

  public RightSinglePickup() {
    super(
        Paths.rightShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.rightShootToPickup),
            Commands.waitTime(Seconds.of(1))
                .andThen(StateMachine.getInstance().setStateCommand(new Intake()))),
        AutoBuilder.followPath(Paths.rightPickupToShoot),
        StateMachine.getInstance().setStateCommand(new Score()));
  }
}
