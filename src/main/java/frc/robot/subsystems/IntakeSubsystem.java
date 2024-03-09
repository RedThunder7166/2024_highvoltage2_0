// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SensorUtils;
import frc.robot.Constants;
import frc.robot.OurRobotState;

// TODO: intake manual mode
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
  private static final DutyCycleOut intakeForwardRequest = new DutyCycleOut(Constants.Intake.INTAKE_PERCENT_OUTPUT);
  private static final DutyCycleOut intakeBackwardRequest = new DutyCycleOut(-Constants.Intake.INTAKE_PERCENT_OUTPUT);

  private final TalonFX m_indexingMotor = new TalonFX(Constants.Intake.INDEXING_MOTOR_ID);
  // TODO: either flip the negative or invert the motor depending on if fork-in-the-road motor directions are flipped
  private static final DutyCycleOut indexingElevatorRequest = new DutyCycleOut(Constants.Intake.INDEXING_PERCENT_OUTPUT);
  private static final DutyCycleOut indexingShooterRequest = new DutyCycleOut(-Constants.Intake.INDEXING_PERCENT_OUTPUT);

  private final DigitalInput m_entranceSensor = new DigitalInput(Constants.Intake.ENTRANCE_SENSOR_ID);
  private final DigitalInput m_exitSensor = new DigitalInput(Constants.Intake.EXIT_SENSOR_ID);

  private boolean m_forwardIntakeState = false;
  private boolean m_backwardIntakeState = false;

  private static enum IndexingState {
    Stop,
    TowardShooter,
    TowardElevator
  };
  private IndexingState m_indexingState = IndexingState.Stop;

  private boolean m_isIntakingForward = false;

  private boolean m_entranceSensorState = false;
  private boolean m_exitSensorState = false;

  public IntakeSubsystem() {
    final TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();

    intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: should intake motor be brake or coast?

    intakeMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.Intake.INTAKE_MOTOR_CURRENT_LIMIT;

    m_intakeMotor.getConfigurator().apply(intakeMotorConfiguration);

    final TalonFXConfiguration indexingMotorConfiguration = new TalonFXConfiguration();

    indexingMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexingMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: should intake indexing motor be brake or coast?

    m_indexingMotor.getConfigurator().apply(indexingMotorConfiguration);
  }

  @Override
  public void periodic() {
    // TODO: check if intake entrance & exit sensors should be flipped
    m_entranceSensorState = SensorUtils.isAllenBradleyDown(m_entranceSensor);
    m_exitSensorState = SensorUtils.isAllenBradleyDown(m_exitSensor);

    boolean isIntakingForward = false;

    if (m_exitSensorState) {
      m_forwardIntakeState = false;
      m_backwardIntakeState = false;
    }

    final OurRobotState.SequenceState sequenceState = OurRobotState.getSequenceState();
    if (sequenceState != OurRobotState.SequenceState.None && OurRobotState.getShootMode() == OurRobotState.ShootMode.Speaker) {
      switch (sequenceState) {
        case Firing:
          m_intakeMotor.setControl(intakeForwardRequest); // note: this is duplicate logic (check the else branch below), so make sure to sync
          isIntakingForward = true;
          break;

        default:
          m_intakeMotor.disable();
          m_indexingState = IndexingState.Stop;
          break;
      }
    } else {
      if (m_backwardIntakeState) {
        m_intakeMotor.setControl(intakeBackwardRequest);
      } else if (m_forwardIntakeState) {
        m_intakeMotor.setControl(intakeForwardRequest);
        isIntakingForward = true;
      } else {
        m_intakeMotor.disable();
      }
    }

    m_isIntakingForward = isIntakingForward;

    switch (m_indexingState) {
      case Stop:
        m_indexingMotor.disable();
        break;
      case TowardElevator:
        m_indexingMotor.setControl(indexingElevatorRequest);
      case TowardShooter:
        m_indexingMotor.setControl(indexingShooterRequest);
      default:
        break;
    }
  }

  public boolean getEntranceSensorState() {
    return m_entranceSensorState;
  }
  public boolean getExitSensorState() {
    return m_exitSensorState;
  }

  public boolean getIsIntakingForward() {
    return m_isIntakingForward;
  }

  public final InstantCommand toggleForwardCommand = new InstantCommand(() -> {
    m_forwardIntakeState = !m_forwardIntakeState;
  }, this);

  public final Command backwardOnOffCommand = Commands.startEnd(() -> {
    m_backwardIntakeState = true;
  }, () -> {
    m_backwardIntakeState = false;
  }, this);

  public void stopIndexing() {
    m_indexingState = IndexingState.Stop;
  }
  public void indexingTowardElevator() {
    m_indexingState = IndexingState.TowardElevator;
  }
  public void indexingTowardShooter() {
    m_indexingState = IndexingState.TowardShooter;
  }
}
