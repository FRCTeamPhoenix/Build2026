// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.TurretConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Turret extends SubsystemBase {

  private final SmartMotorIO turretMotor;
  private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();
  private final Alert motorAlert = new Alert("Turret motor is disconnected!", AlertType.kError);
  private double goal = TurretConstants.STARTING_ANGLE;

  public Turret(SmartMotorIO turretMotor) {
    this.turretMotor = turretMotor;
    setName("Shooter/Turret");
    turretMotor.setPosition(TurretConstants.STARTING_ANGLE);
    setDefaultCommand(run(() -> turretMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    turretMotor.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);
    motorAlert.set(!inputs.motorsConnected[0]);

    ExecutionLogger.log("Shooter/Turret");
  }

  public void runPosition(Rotation2d target) {
    this.goal = calculateTurretAngle(target);
    turretMotor.runPosition(goal);
  }

  public void runPosition(Supplier<Rotation2d> target) {
    this.goal = calculateTurretAngle(target.get());
    turretMotor.runPosition(goal);
  }

  public Command runPositionCommand(Rotation2d target) {
    return run(() -> runPosition(target)).withName("Turret RunPosition");
  }

  public Command runPositionCommand(Supplier<Rotation2d> target) {
    return run(() -> runPosition(target)).withName("Turret RunPosition");
  }

  public Command goToPositionCommand(Rotation2d target) {
    return run(() -> runPosition(target))
        .until(() -> Math.abs(inputs.positionRad - goal) <= TurretConstants.AT_POSITION_THRESHOLD)
        .withName("Turret GoToPosition");
  }

  public Command goToPositionCommand(Supplier<Rotation2d> target) {
    return run(() -> runPosition(target))
        .until(() -> Math.abs(inputs.positionRad - goal) <= TurretConstants.AT_POSITION_THRESHOLD)
        .withName("Turret GoToPosition");
  }

  public Command runVoltage(double voltage) {
    return run(() -> turretMotor.runVoltage(voltage)).withName("Turret Voltage");
  }

  public Command stop() {
    return runOnce(() -> turretMotor.runVoltage(0.0));
  }

  @AutoLogOutput(key = "Shooter/Turret/Position")
  public Rotation2d getTurretPosition() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  public double getTurretVelocity() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return new Rotation2d(goal);
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionRad - goal) <= TurretConstants.AT_POSITION_THRESHOLD;
  }

  public void zeroTurret() {
    turretMotor.setPosition(0.0);
  }

  private double calculateTurretAngle(Rotation2d angle) {
    double calculatedAngle = MathUtil.inputModulus(angle.getRadians(), -Math.PI, Math.PI);
    if (calculatedAngle < 0) calculatedAngle += 2 * Math.PI;
    if (calculatedAngle > Math.PI * 2) calculatedAngle -= 2 * Math.PI;
    return MathUtil.clamp(
        calculatedAngle, TurretConstants.MIN_TURRET_ANGLE, TurretConstants.MAX_TURRET_ANGLE);
  }
}
