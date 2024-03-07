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
  public LEDSubsystem(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    m_elevator = elevator;
    m_intake = intake;
  }

  @Override
  public void periodic() {
    OurRobotState.LEDValue ledValue = LEDValue.Idle;
    if (m_elevator.getIsAtTop()) {
      ledValue = LEDValue.SolidGreen;
    } else if (m_intake.getIsIntakingForward()) {
      ledValue = LEDValue.Color1Flash;
    }

    m_blinkIn.set(ledValue.value);
  }
}
