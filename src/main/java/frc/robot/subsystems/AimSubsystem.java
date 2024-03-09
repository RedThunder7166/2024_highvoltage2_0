// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.SequenceState;

public class AimSubsystem extends SubsystemBase {
  private final VisionSubsystem m_vision;

  private final TalonFX m_angleMotor = new TalonFX(Constants.Aim.ANGLE_MOTOR_ID);
  private final DutyCycleOut m_angleMotorRequest = new DutyCycleOut(0);

  private final CANcoder m_angleEncoder = new CANcoder(Constants.Aim.ANGLE_CANCODER_ID);

  private double m_lastAngleEncoderPosition = 0;

  private final PIDController m_aimPIDController = new PIDController(
    0.15,
    0,
    0
  );
  private final double m_aimMaxSpeed = 1.5;

  public AimSubsystem(VisionSubsystem vision) {
    m_vision = vision;

    final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_angleMotor.getConfigurator().apply(motorConfiguration);
  }

  @Override
  public void periodic() {
    final double angleEncoderPosition = m_angleEncoder.getAbsolutePosition().getValueAsDouble();
    if (m_lastAngleEncoderPosition != angleEncoderPosition) {
      m_angleMotor.setPosition(angleEncoderPosition * Constants.Aim.ANGLE_MOTOR_GEAR_RATIO);
    }
    m_lastAngleEncoderPosition = angleEncoderPosition;

    final OurRobotState.SequenceState sequenceState = OurRobotState.getSequenceState();
    if (OurRobotState.getShootMode() == OurRobotState.ShootMode.Speaker) {
      switch (sequenceState) {
        case None:
          m_angleMotor.disable();
          break;

        case Initiated:
        case Firing:
          autoTarget();
          break;

        default:
          break;
      }
    } else if (sequenceState == SequenceState.Initiated) { // TODO: check if collision avoidance should only apply to climb / trap or if it should apply to both climb / trap and amp
      aimAtPosition(Constants.Aim.COLLISION_AVOIDANCE_POSITION);
    } else {
      m_angleMotor.disable();
    }
  }

  public void aimAtPosition(double position) {
    double value = m_aimPIDController.calculate(m_angleMotor.getPosition().getValueAsDouble(), position) * m_aimMaxSpeed;
    if (Math.abs(m_aimPIDController.getPositionError()) < Constants.Aim.MINIMUM_PID_ERROR) {
      value = 0;
    }
    m_angleMotor.setControl(m_angleMotorRequest.withOutput(value));
  }
  public void autoTarget() {
    aimAtPosition(m_vision.getAutoTargetPosition());
  }
}
