// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
  private static final DutyCycleOut m_intakeForwardRequest = new DutyCycleOut(Constants.Intake.INTAKE_PERCENT_OUTPUT);
  private static final DutyCycleOut m_intakeBackwardRequest = new DutyCycleOut(-Constants.Intake.INTAKE_PERCENT_OUTPUT);

  private boolean m_forwardIntakeState = false;
  private boolean m_backwardIntakeState = false;

  private boolean m_isIntakingForward = false;

  public IntakeSubsystem() {
    final TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();

    intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.Intake.INTAKE_MOTOR_CURRENT_LIMIT;

    m_intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean isIntakingForward = false;
    if (m_backwardIntakeState) {
      m_intakeMotor.setControl(m_intakeBackwardRequest);
    } else if (m_forwardIntakeState) {
      m_intakeMotor.setControl(m_intakeForwardRequest);
      isIntakingForward = true;
    } else {
      m_intakeMotor.disable();
    }

    m_isIntakingForward = isIntakingForward;
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
}
