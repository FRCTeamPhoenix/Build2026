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
  private final DumbMotorIO beltMotor;
  private final DumbMotorIO feederMotor;
  private final DumbMotorIOInputsAutoLogged beltMotorInputs = new DumbMotorIOInputsAutoLogged();
  private final DumbMotorIOInputsAutoLogged feederMotorInputs = new DumbMotorIOInputsAutoLogged();

  private final Alert beltMotorAlert =
      new Alert("Indexer Belt Motor is diconnected", AlertType.kError);
  private final Alert feederMotorAlert =
      new Alert("Indexer Feeder Motor is diconnected", AlertType.kError);

  public Indexer(DumbMotorIO beltMotor, DumbMotorIO feederMotor) {
    this.beltMotor = beltMotor;
    this.feederMotor = feederMotor;
    setName("Indexer");

    setDefaultCommand(
        run(
            () -> {
              beltMotor.runVoltage(0.0);
              feederMotor.runVoltage(0.0);
            }));
  }

  @Override
  public void periodic() {
    beltMotor.updateInputs(beltMotorInputs);
    feederMotor.updateInputs(feederMotorInputs);

    Logger.processInputs("Indexer/BeltMotor", beltMotorInputs);
    Logger.processInputs("Indexer/FeederMotor", feederMotorInputs);

    beltMotorAlert.set(!beltMotorInputs.connected);
    feederMotorAlert.set(!feederMotorInputs.connected);

    ExecutionLogger.log("Indexer");
  }

  public Command feed() {
    return run(() -> {
          beltMotor.runTorqueCurrent(IndexerConstants.RUN_CURRENT);
          feederMotor.runTorqueCurrent(IndexerConstants.RUN_CURRENT);
        })
        .withName("Indexer Feed");
  }

  public Command out() {
    return run(() -> {
          beltMotor.runTorqueCurrent(-IndexerConstants.RUN_CURRENT);
          feederMotor.runTorqueCurrent(-IndexerConstants.RUN_CURRENT);
        })
        .withName("Indexer Out");
  }

  public Command stop() {
    return runOnce(
            () -> {
              beltMotor.runVoltage(0.0);
              feederMotor.runVoltage(0.0);
            })
        .withName("Indexer Stop");
  }
}
