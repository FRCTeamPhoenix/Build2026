// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lombok.experimental.Delegate;
import org.team2342.frc.Constants;
import org.team2342.frc.subsystems.shooter.Flywheel;
import org.team2342.frc.subsystems.shooter.Turret;
import org.team2342.frc.util.FiringSolver;
import org.team2342.lib.fsm.StateMachine;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.logging.tunable.TunableNumber;

public class Conductor extends SubsystemBase {

  private TunableNumber flywheelSpeed =
      new TunableNumber("FlywheelSpeedMPS", 0, () -> Constants.TUNING);

  public enum ConductorState {
    UNDETERMINED,
    DISABLED,
    WARM_UP,
    TRACKED_FIRING,
    TUNING,
  }

  @Delegate(types = FSMDelegate.class)
  private final StateMachine<ConductorState> fsm =
      new StateMachine<ConductorState>(
          "Conductor",
          ConductorState.UNDETERMINED,
          () -> ConductorState.DISABLED,
          ConductorState.class);

  private final Flywheel flywheel;
  private final Turret turret;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;

  public Conductor(
      Flywheel flywheel,
      Turret turret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier) {
    this.flywheel = flywheel;
    this.turret = turret;

    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;

    setupStateCommands();
    setupTransitions();

    fsm.enable();

    setDefaultCommand(runState(ConductorState.DISABLED));
  }

  @Override
  public void periodic() {
    fsm.periodic();
    ExecutionLogger.log("Conductor");
  }

  public Command runState(ConductorState state) {
    return run(() -> fsm.requestTransition(state)).withName("Conductor Run " + state.name());
  }

  public Command goToState(ConductorState state) {
    return fsm.requestTransitionCommand(state).withName("Conductor Switch To " + state.name());
  }

  public Command waitForState(ConductorState state) {
    return fsm.waitForState(state).withName("Conductor Wait For " + state.name());
  }

  private void setupStateCommands() {
    fsm.addStateCommand(ConductorState.DISABLED, Commands.parallel(turret.stop(), flywheel.stop()));

    fsm.addStateCommand(
        ConductorState.WARM_UP,
        turret
            .runPositionCommand(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .turretAngle())
            .alongWith(flywheel.warmUp()));

    fsm.addStateCommand(
        ConductorState.TRACKED_FIRING,
        turret
            .runPositionCommand(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .turretAngle())
            .alongWith(
                flywheel.shoot(
                    () ->
                        FiringSolver.getInstance()
                            .calculate(velocitySupplier.get(), poseSupplier.get())
                            .wheelSpeed())));

    fsm.addStateCommand(
        ConductorState.TUNING,
        turret
            .runPositionCommand(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .turretAngle())
            .alongWith(flywheel.shoot(flywheelSpeed)));
  }

  public Command disable() {
    return Commands.runOnce(() -> fsm.disable());
  }

  public Command enable() {
    return Commands.runOnce(() -> fsm.enable());
  }

  public ConductorState getCurrentState() {
    return fsm.getCurrentState();
  }

  private void setupTransitions() {
    fsm.addOmniTransition(ConductorState.DISABLED);
    fsm.addOmniTransition(ConductorState.TRACKED_FIRING);
    fsm.addOmniTransition(ConductorState.WARM_UP);
    fsm.addOmniTransition(ConductorState.TUNING);
  }

  public interface FSMDelegate {
    String dot();

    boolean isEnabled();
  }
}
