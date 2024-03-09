// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.LEDValue;

public class LEDSubsystem extends SubsystemBase {
  private final Spark m_blinkIn = new Spark(Constants.LED.BLINKIN_ID);

  private final ElevatorSubsystem m_elevator;
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;
  private final AmpTrapSubsystem m_amptrap;
  public LEDSubsystem(ElevatorSubsystem elevator, IntakeSubsystem intake, ShooterSubsystem shooter, AmpTrapSubsystem amptrap) {
    m_elevator = elevator;
    m_intake = intake;
    m_shooter = shooter;
    m_amptrap = amptrap;
  }

  // checks not only if elevator is at the top, but also if we are in a mode that moves the elevator
  private boolean isElevatorAtTop() {
    switch (OurRobotState.getShootMode()) {
      case Amp:
      case Climb_Trap:
        return m_elevator.getIsAtTop();
    
      default:
        return false;
    }
  }

  @Override
  public void periodic() {
    OurRobotState.LEDValue ledValue = LEDValue.Idle;
    if (m_intake.getEntranceSensorState()) {
      ledValue = LEDValue.Color2Flash;
    } else if (m_intake.getExitSensorState() || isElevatorAtTop() || m_shooter.getIsFiring()) {
      ledValue = LEDValue.SolidGreen;
    } else if (m_shooter.getIsTargetInRange()) {
      ledValue = LEDValue.Color2Flash;
    } else if (m_intake.getIsIntakingForward() || m_shooter.getIsTargetingAndEngaged() || m_amptrap.getIsInMotion()) {
      ledValue = LEDValue.Color1Flash;
    }

    m_blinkIn.set(ledValue.value);
  }
}
