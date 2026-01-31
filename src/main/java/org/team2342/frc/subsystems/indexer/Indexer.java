// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IndexerConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Indexer extends SubsystemBase {
  private final DumbMotorIO motor1;
  private final DumbMotorIO motor2;
  private final DumbMotorIO motor3;
  private final DumbMotorIOInputsAutoLogged motor1Inputs = new DumbMotorIOInputsAutoLogged();
  private final DumbMotorIOInputsAutoLogged motor2Inputs = new DumbMotorIOInputsAutoLogged();
  private final DumbMotorIOInputsAutoLogged motor3Inputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert1 = new Alert("Indexer Motor 1 is diconnected", AlertType.kError);
  private final Alert motorAlert2 = new Alert("Indexer Motor 2 is diconnected", AlertType.kError);
  private final Alert motorAlert3 = new Alert("Indexer Motor 3 is diconnected", AlertType.kError);

  public Indexer(DumbMotorIO motor1, DumbMotorIO motor2, DumbMotorIO motor3) {
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.motor3 = motor3;
    setName("Indexer");

    setDefaultCommand(
        run(
            () -> {
              motor1.runVoltage(0.0);
              motor2.runVoltage(0.0);
              motor3.runVoltage(0.0);
            }));
  }

  @Override
  public void periodic() {
    motor1.updateInputs(motor1Inputs);
    motor2.updateInputs(motor2Inputs);
    motor3.updateInputs(motor3Inputs);
    Logger.processInputs("Indexer/Motor1", motor1Inputs);
    Logger.processInputs("Indexer/Motor2", motor2Inputs);
    Logger.processInputs("Indexer/Motor3", motor3Inputs);

    motorAlert1.set(!motor1Inputs.connected);
    motorAlert2.set(!motor2Inputs.connected);
    motorAlert3.set(!motor3Inputs.connected);

    ExecutionLogger.log("Indexer");
  }

  public Command out() {
    return run(() -> {
          motor1.runVoltage(IndexerConstants.RUN_VOLTAGE);
          motor2.runVoltage(IndexerConstants.RUN_VOLTAGE);
          motor3.runVoltage(IndexerConstants.RUN_VOLTAGE);
        })
        .withName("Indexer Out");
  }

  public Command in() {
    return run(() -> {
          motor1.runVoltage(-IndexerConstants.RUN_VOLTAGE);
          motor2.runVoltage(-IndexerConstants.RUN_VOLTAGE);
          motor3.runVoltage(-IndexerConstants.RUN_VOLTAGE);
        })
        .withName("Indexer In");
  }

  public Command stop() {
    return runOnce(
            () -> {
              motor1.runVoltage(0.0);
              motor2.runVoltage(0.0);
              motor3.runVoltage(0.0);
            })
        .withName("Indexer Stop");
  }
}
