// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;

public class AimSubsystem extends SubsystemBase {
  private final VisionSubsystem m_vision;
  public AimSubsystem(VisionSubsystem vision) {
    m_vision = vision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final OurRobotState.ShootMode shootMode = OurRobotState.getShootMode();
    final double targetPosition;
    if (shootMode.m_autoAim) {
      targetPosition = m_vision.getAutoAimPosition();
    } else {
      targetPosition = shootMode.m_position;
    }


  }
}
