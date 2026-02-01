// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Flywheel extends SubsystemBase {

  private final SmartMotorIO wheelMotor;
  private final SmartMotorIOInputsAutoLogged wheelMotorInputs = new SmartMotorIOInputsAutoLogged();

  public Flywheel(SmartMotorIO wheelMotor) {
    this.wheelMotor = wheelMotor;
    setName("Shooter/Flywheel");
    setDefaultCommand(run(() -> wheelMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    wheelMotor.updateInputs(wheelMotorInputs);
    Logger.processInputs("Shooter/Flywheel", wheelMotorInputs);
  }

  public void runVelocity(double metersPerSec) {
    double radPerSec = metersPerSec / ShooterConstants.WHEEL_RADIUS_METERS;
    Logger.recordOutput("Shooter/Flywheel/SetpointMetersPerSec", metersPerSec);
    wheelMotor.runVelocity(radPerSec);
  }
  public Command shoot(double metersPerSec) {
    return run(() -> runVelocity(metersPerSec)).withName("Run Shooter");
  }

  public Command stop() {
    return runOnce(() -> wheelMotor.runVoltage(0.0)).withName("Shooter Stop");
  }

  @AutoLogOutput(key = "Shooter/Flywheel/VelocityMetersPerSec")
  public double getVelocityMetersPerSec() {
    return wheelMotorInputs.velocityRadPerSec * ShooterConstants.WHEEL_RADIUS_METERS;
  }
}
