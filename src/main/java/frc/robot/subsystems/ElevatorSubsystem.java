// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  // right follows left
  private final TalonFX m_leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
  private final TalonFX m_rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

  private boolean m_isAtTop;

  private final DigitalInput m_bottomBumperSensor = new DigitalInput(Constants.Elevator.BOTTOM_BUMPER_SENSOR_ID);

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Elevator");

  private final BooleanPublisher m_bottomBumperSensorPublisher = m_table.getBooleanTopic("BottomBumperSensor").publish();

  public ElevatorSubsystem() {
    final TalonFXConfiguration leftRightConfiguration = new TalonFXConfiguration();

    leftRightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leftMotor.getConfigurator().apply(leftRightConfiguration);
    m_rightMotor.getConfigurator().apply(leftRightConfiguration);

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), Constants.Elevator.RIGHT_MOTOR_OPPOSES_LEFT));
  }

  @Override
  public void periodic() {
    // TODO: possibly invert this: check if true means pressed or not. the value is shown in network tables, so just take a peek
    final boolean bottomBumperSensorPressed = m_bottomBumperSensor.get();
    m_bottomBumperSensorPublisher.set(bottomBumperSensorPressed);

    if (bottomBumperSensorPressed) {
      m_leftMotor.setPosition(0);
      m_rightMotor.setPosition(0);
    }

    m_isAtTop = m_leftMotor.getPosition().getValueAsDouble() >= Constants.Elevator.TOP_POSITION;
  }

  public boolean getIsAtTop() {
    return m_isAtTop;
  }
}
