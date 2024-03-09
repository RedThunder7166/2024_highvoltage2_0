// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAutoTargetPosition() {
    // TODO: implement auto target position
    return 0;
  }
  public double getAutoTargetSpeed() {
    // TODO: implement auto target speed
    return 0;
  }
  public boolean getIsSpeakerAutoTargetInRange() {
    // TODO: implement auto target
    // this should just be if front camera sees AllianceColor.SUBWOOFER_CENTER
    // and maybe some additional clarity checks
    return false;
  }
}
