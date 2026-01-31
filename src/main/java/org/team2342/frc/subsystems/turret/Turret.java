// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.TurretConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Turret extends SubsystemBase {

  private final SmartMotorIO turretMotor;
  private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();
  private Rotation2d goal = new Rotation2d();

  public Turret(SmartMotorIO turretMotor) {
    this.turretMotor = turretMotor;
    setName("Turret");
    setDefaultCommand(run(() -> turretMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    turretMotor.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    ExecutionLogger.log("Turret");
  }

  public void runPosition(Rotation2d setpoint) {
    this.goal = setpoint.minus(Rotation2d.kZero);
    turretMotor.runPosition(this.goal.getRadians());
  }

  public Command runPositionCommand(Rotation2d setpoint) {
    return run(() -> runPosition(setpoint)).withName("Turret RunPosition");
  }

  public void goToPosition(Rotation2d setpoint) {
    this.goal = setpoint.minus(Rotation2d.kZero);
    turretMotor.runPosition(this.goal.getRadians());
  }

  public Command goToPositionCommand(Rotation2d setpoint) {
    return run(() -> goToPosition(setpoint))
        .until(
            () ->
                Math.abs(turretPosition().minus(goal).getRadians())
                    <= TurretConstants.AT_POSITION_THRESHOLD)
        .withName("Turret GoToPosition");
  }

  public Command stop() {
    return runOnce(() -> turretMotor.runVoltage(0.0));
  }

  @AutoLogOutput(key = "Turret/Position")
  public Rotation2d turretPosition() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  public double turretVelocity() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput(key = "Turret/Setpoint")
  public Rotation2d turretSetpoint() {
    return goal;
  }

  public void zeroTurret() {
    turretMotor.setPosition(0.0);
  }
}
