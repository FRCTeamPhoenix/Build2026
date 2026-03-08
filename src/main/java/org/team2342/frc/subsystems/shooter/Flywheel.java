// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Flywheel extends SubsystemBase {

  private final SmartMotorIO motor;
  private final SmartMotorIOInputsAutoLogged motorInputs = new SmartMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Flywheel motor is disconnected!", AlertType.kError);

  private final SysIdRoutine sysId;

  public Flywheel(SmartMotorIO motor) {
    this.motor = motor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> motor.runVoltage(voltage.in(Volts)),
                (log) ->
                    Logger.recordOutput("Shooter/Flywheel/Voltage", motorInputs.appliedVolts[0]),
                this));

    setName("Shooter/Flywheel");
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Shooter/Flywheel", motorInputs);

    motorAlert.set(!motorInputs.motorsConnected[0]);

    ExecutionLogger.log("Shooter/Flywheel");
  }

  public void runVelocity(double metersPerSec) {
    double radPerSec = metersPerSec / ShooterConstants.FLYWHEEL_RADIUS_METERS;
    Logger.recordOutput("Shooter/Flywheel/SetpointMetersPerSec", metersPerSec);
    motor.runVelocity(radPerSec);
  }

  public void runVelocity(DoubleSupplier metersPerSec) {
    double radPerSec = metersPerSec.getAsDouble() / ShooterConstants.FLYWHEEL_RADIUS_METERS;
    Logger.recordOutput("Shooter/Flywheel/SetpointMetersPerSec", metersPerSec);
    motor.runVelocity(radPerSec);
  }

  private void warmUp(double idleSpeed) {
    if (getVelocityMetersPerSec() > idleSpeed) {
      motor.runVoltage(0.0);
    } else {
      runVelocity(idleSpeed);
    }
  }

  public Command shoot(double metersPerSec) {
    return run(() -> runVelocity(metersPerSec)).withName("Run Shooter");
  }

  public Command shoot(DoubleSupplier metersPerSec) {
    return run(() -> runVelocity(metersPerSec)).withName("Run Shooter");
  }

  public Command warmUp() {
    return run(() -> warmUp(ShooterConstants.IDLE_SPEED)).withName("Warm Up Shooter");
  }

  public Command runVoltage(double volts) {
    return run(() -> runVoltage(volts)).withName("Run Shooter Volts");
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Shooter Stop");
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction))
        .withName("Flywheel Quasistatic");
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction))
        .withName("Flywheel Dynamic");
  }

  @AutoLogOutput(key = "Shooter/Flywheel/VelocityMetersPerSec")
  public double getVelocityMetersPerSec() {
    return motorInputs.velocityRadPerSec * ShooterConstants.FLYWHEEL_RADIUS_METERS;
  }
}
