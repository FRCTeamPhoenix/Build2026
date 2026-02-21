// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lombok.experimental.Delegate;
import org.team2342.frc.subsystems.shooter.Flywheel;
import org.team2342.frc.subsystems.shooter.Hood;
import org.team2342.frc.util.FiringSolver;
import org.team2342.lib.fsm.StateMachine;
import org.team2342.lib.logging.ExecutionLogger;

public class Conductor extends SubsystemBase {

  public enum ConductorState {
    UNDETERMINED,
    DISABLED,
    TRENCH,
    BUMPER_SHOT,
    WARM_UP,
    TRACKED_FIRING,
    OVERRIDE_25,
    OVERRIDE_23;
  }

  @Delegate(types = FSMDelegate.class)
  private final StateMachine<ConductorState> fsm =
      new StateMachine<ConductorState>(
          "Conductor",
          ConductorState.UNDETERMINED,
          () -> ConductorState.DISABLED,
          ConductorState.class);

  private final Flywheel flywheel;
  private final Hood hood;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;

  public Conductor(
      Flywheel flywheel,
      Hood hood,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier) {
    this.flywheel = flywheel;
    this.hood = hood;

    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;

    setupStateCommands();
    setupTransitions();

    enable();

    setDefaultCommand(runState(ConductorState.DISABLED));
  }

  @Override
  public void periodic() {
    fsm.periodic();
    ExecutionLogger.log("Conductor");
  }

  public Command runState(ConductorState state) {
    return run(() -> fsm.requestTransition(state));
  }

  public Command goToState(ConductorState state) {
    return fsm.requestTransitionCommand(state).withName("Conductor Switch To " + state.name());
  }

  public Command waitForState(ConductorState state) {
    return fsm.waitForState(state).withName("Conductor Wait For " + state.name());
  }

  private void setupStateCommands() {
    fsm.addStateCommand(ConductorState.DISABLED, hood.stop().alongWith(flywheel.stop()));

    fsm.addStateCommand(ConductorState.TRENCH, hood.holdAngle(0.0));

    fsm.addStateCommand(
        ConductorState.BUMPER_SHOT,
        flywheel
            .shoot(FiringSolver.BUMPER_SHOT.wheelSpeed())
            .alongWith(hood.holdAngle(FiringSolver.BUMPER_SHOT.hoodAngle())));

    fsm.addStateCommand(
        ConductorState.WARM_UP,
        hood.holdAngle(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .hoodAngle())
            .deadlineFor(flywheel.warmUp()));

    fsm.addStateCommand(
        ConductorState.TRACKED_FIRING,
        hood.holdAngle(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .hoodAngle())
            .deadlineFor(
                flywheel.shoot(
                    () ->
                        FiringSolver.getInstance()
                            .calculate(velocitySupplier.get(), poseSupplier.get())
                            .wheelSpeed())));

    fsm.addStateCommand(ConductorState.OVERRIDE_25, flywheel.shoot(25.0));

    fsm.addStateCommand(ConductorState.OVERRIDE_23, flywheel.shoot(23.0));
  }

  private void setupTransitions() {
    fsm.addOmniTransition(ConductorState.DISABLED);

    fsm.addOmniTransition(ConductorState.TRENCH, hood.goToAngle(0.0));

    fsm.addOmniTransition(
        ConductorState.BUMPER_SHOT,
        hood.goToAngle(FiringSolver.BUMPER_SHOT.hoodAngle())
            .deadlineFor(flywheel.shoot(FiringSolver.BUMPER_SHOT.wheelSpeed())));

    fsm.addOmniTransition(
        ConductorState.WARM_UP,
        hood.goToAngle(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .hoodAngle())
            .deadlineFor(flywheel.warmUp()));

    fsm.addOmniTransition(
        ConductorState.TRACKED_FIRING,
        hood.goToAngle(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .hoodAngle())
            .deadlineFor(
                flywheel.shoot(
                    () ->
                        FiringSolver.getInstance()
                            .calculate(velocitySupplier.get(), poseSupplier.get())
                            .wheelSpeed())));
    fsm.addOmniTransition(
        ConductorState.OVERRIDE_25,
        hood.goToAngle(
                () ->
                    FiringSolver.getInstance()
                        .calculate(velocitySupplier.get(), poseSupplier.get())
                        .hoodAngle())
            .deadlineFor(flywheel.shoot(25.0)));

    fsm.addOmniTransition(ConductorState.OVERRIDE_23, flywheel.shoot(23.0));
  }

  public Command makeShooterCommand(double flywheelMetersPerSec, double hoodAngle) {
    return hood.holdAngle(hoodAngle).alongWith(flywheel.shoot(flywheelMetersPerSec));
  }

  public interface FSMDelegate {
    String dot();

    void enable();

    void disable();

    boolean isEnabled();
  }
}
