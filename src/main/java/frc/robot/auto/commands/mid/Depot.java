package frc.robot.auto.commands.mid;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.state.StateMachine;
import frc.robot.state.standard.Intake;
import frc.robot.state.standard.Score;

public class Depot extends BaseAuto {

  public Depot() {
    super(
        Paths.midShootToDepotPickup.getStartingHolonomicPose().orElseThrow(),
        StateMachine.getInstance().setStateCommand(new Intake()),
        Commands.parallel(
            StateMachine.getInstance().setStateCommand(new Score()),
            Commands.sequence(
                AutoBuilder.followPath(Paths.midShootToDepotPickup),
                AutoBuilder.followPath(Paths.depotPickupToShoot))));
  }
}
