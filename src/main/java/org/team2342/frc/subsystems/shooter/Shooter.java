// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

  private final SmartMotorIO wheelMotor;
  private final SmartMotorIOInputsAutoLogged wheelMotorInputs = new SmartMotorIOInputsAutoLogged();

  public Shooter(SmartMotorIO wheelMotor) {
    this.wheelMotor = wheelMotor;
    setName("Shooter");
    setDefaultCommand(run(() -> wheelMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    wheelMotor.updateInputs(wheelMotorInputs);
    Logger.processInputs("Shooter/WheelMotor", wheelMotorInputs);
  }

  public Command shoot(double metersPerSec) {
    double radPerSec = metersPerSec / ShooterConstants.WHEEL_RADIUS_METERS;
    return run(() -> wheelMotor.runVelocity(radPerSec)).withName("Run Shooter");
  }

  public Command stop() {
    return runOnce(() -> wheelMotor.runVoltage(0.0)).withName("Shooter Stop");
  }

  public double getVelocityMetersPerSec() {
    return wheelMotorInputs.velocityRadPerSec * ShooterConstants.WHEEL_RADIUS_METERS;
  }
}
