// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import lombok.experimental.Delegate;
import org.team2342.frc.subsystems.shooter.Flywheel;
import org.team2342.frc.subsystems.shooter.Hood;
import org.team2342.frc.util.FiringSolver;
import org.team2342.frc.util.FiringSolver.FiringSolution;
import org.team2342.lib.fsm.StateMachine;

public class Superstructure implements Subsystem {

  public enum SuperstructureState {
    UNDETERMINED,
    DISABLED,
    TRENCH,
    BUMPERSHOT,
    TRACKING;
  }

  @Delegate(types = FSMDelegate.class)
  private final StateMachine<SuperstructureState> superstructureFSM =
      new StateMachine<SuperstructureState>(
          "Superstructure",
          SuperstructureState.UNDETERMINED,
          () -> SuperstructureState.DISABLED,
          SuperstructureState.class);

  private final Flywheel flywheel;
  private final Hood hood;

  public Superstructure(Flywheel flywheel, Hood hood) {
    this.flywheel = flywheel;
    this.hood = hood;

    setupStateCommands();
    setupTransitions();

    enable();
  }

  public Command goToState(SuperstructureState state) {
    return superstructureFSM
        .requestTransitionCommand(state)
        .withName("Superstructure Switch To " + state.name());
  }

  public Command waitForState(SuperstructureState state) {
    return superstructureFSM
        .waitForState(state)
        .withName("Superstructure Wait For " + state.name());
  }

  private void setupStateCommands() {
    superstructureFSM.addStateCommand(SuperstructureState.TRENCH, null);
    superstructureFSM.addStateCommand(
        SuperstructureState.BUMPERSHOT,
        makeShooterCommand(
            FiringSolver.BUMPERSHOT.wheelSpeed(), FiringSolver.BUMPERSHOT.hoodAngle()));
    superstructureFSM.addStateCommand(SuperstructureState.TRENCH, hood.goToAngle(0.0));
    superstructureFSM.addStateCommand(
        SuperstructureState.TRACKING,
        makeShooterCommand(() -> FiringSolver.getInstance().calculate(null, null)));
  }

  private void setupTransitions() {
    superstructureFSM.addOmniTransition(SuperstructureState.DISABLED);
    // superstructureFSM.addOmniTransition(
    //     SuperstructureState.L1, null);
  }

  public Command makeShooterCommand(double flywheelMetersPerSec, double hoodAngle) {
    return hood.holdAngle(hoodAngle).alongWith(flywheel.shoot(flywheelMetersPerSec));
  }

  private Command makeShooterCommand(Supplier<FiringSolution> solutionSupplier) {
    return hood.holdAngle(solutionSupplier.get().hoodAngle())
        .alongWith(flywheel.shoot(solutionSupplier.get().wheelSpeed()));
  }

  public interface FSMDelegate {
    String dot();

    void enable();

    void disable();

    boolean isEnabled();

    void periodic();
  }
}
