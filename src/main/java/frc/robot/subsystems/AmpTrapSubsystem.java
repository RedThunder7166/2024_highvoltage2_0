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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SensorUtils;
import frc.robot.Constants;
import frc.robot.OurRobotState;

public class AmpTrapSubsystem extends SubsystemBase {
  private final TalonFX m_rollerMotor = new TalonFX(Constants.AmpTrap.ROLLER_MOTOR_ID);
  private final DutyCycleOut m_request = new DutyCycleOut(Constants.AmpTrap.PERCENT_OUTPUT);

  private final DigitalInput m_topSensor = new DigitalInput(Constants.AmpTrap.TOP_SENSOR_ID);

  private boolean m_isInMotion = false;

  public AmpTrapSubsystem() {
    final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_rollerMotor.getConfigurator().apply(motorConfiguration);
  }

  @Override
  public void periodic() {
    boolean isInMotion = false;
    final boolean isTopSensorDown = SensorUtils.isAllenBradleyDown(m_topSensor);

    final OurRobotState.SequenceState sequenceState = OurRobotState.getSequenceState();
    switch (OurRobotState.getShootMode()) {
      case Amp:
        switch (sequenceState) {
          case None:
            m_rollerMotor.disable();
            break;

          case Initiated:
            if (isTopSensorDown) {
              m_rollerMotor.setControl(m_request);
              isInMotion = true;
            } else {
              m_rollerMotor.disable();
            }
            break;

          case Firing:
            m_rollerMotor.setControl(m_request);
            break;
        }
        break;

      case Climb_Trap:
        m_rollerMotor.disable(); // TODO: amp/trap climp trap mode
        break;

      case Speaker:
        m_rollerMotor.disable();
        break;
    }

    m_isInMotion = isInMotion;
  }

  public boolean getIsInMotion() {
    return m_isInMotion;
  }
}
