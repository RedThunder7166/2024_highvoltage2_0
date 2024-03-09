// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SensorUtils;
import frc.robot.Constants;
import frc.robot.OurRobotState;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_upperMotor = new TalonFX(Constants.Shooter.UPPER_SHOOTER_MOTOR_ID);
    private final TalonFX m_lowerMotor = new TalonFX(Constants.Shooter.LOWER_SHOOTER_MOTOR_ID);
    private static final DutyCycleOut request = new DutyCycleOut(Constants.Shooter.PERCENT_OUTPUT);

    private final DigitalInput m_exitSensor = new DigitalInput(Constants.Shooter.EXIT_SENSOR_ID);

    private final VisionSubsystem m_vision;

    private boolean m_isTargetingAndEngaged = false;
    private boolean m_isTargetInRange = false;
    private boolean m_isFiring = false;

    public ShooterSubsystem(VisionSubsystem vision) {
        m_vision = vision;

        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: should shooter motor be brake or coast?

        m_upperMotor.getConfigurator().apply(motorConfiguration);
        m_lowerMotor.getConfigurator().apply(motorConfiguration);

        m_lowerMotor.setControl(new Follower(m_upperMotor.getDeviceID(), Constants.Shooter.LOWER_MOTOR_OPPOSES_UPPER));
    }

    @Override
    public void periodic() {
        boolean isTargetingAndEngaged = false;
        boolean isTargetInRange = false;
        boolean isFiring = false;

        final OurRobotState.SequenceState sequenceState = OurRobotState.getSequenceState();
        if (OurRobotState.getShootMode() == OurRobotState.ShootMode.Speaker) {
            switch (sequenceState) {
            case None:
                m_upperMotor.disable();
                break;

            case Initiated:
                autoTarget();
                isTargetingAndEngaged = true;
                isTargetInRange = m_vision.getIsSpeakerAutoTargetInRange();
                break;

            case Firing:
                autoTarget();
                isFiring = true;
                if (SensorUtils.isAllenBradleyDown(m_exitSensor)) {
                    OurRobotState.sequenceReset();
                }
                break;

            default:
                break;
            }
        } else {
            m_upperMotor.disable();
        }

        m_isTargetingAndEngaged = isTargetingAndEngaged;
        m_isTargetInRange = isTargetInRange;
        m_isFiring = isFiring;
    }

    private void autoTarget() {
        m_upperMotor.setControl(request.withOutput(m_vision.getAutoTargetSpeed()));
    }

    public boolean getIsTargetingAndEngaged() {
        return m_isTargetingAndEngaged;
    }
    public boolean getIsTargetInRange() {
        return m_isTargetInRange;
    }
    public boolean getIsFiring() {
        return m_isFiring;
    }
}
