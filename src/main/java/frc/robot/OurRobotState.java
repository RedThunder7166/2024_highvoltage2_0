// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public final class OurRobotState {
    public static enum ShootMode {
        // TODO: get ShootMode shooter speeds
        Amp(0),
        Climb_Trap(0),
        Speaker(0);

        private double m_shooterSpeed;

        private ShootMode(double shooter_speed) {
            m_shooterSpeed = shooter_speed;
        }

        // TODO: ShootMode.Speaker.setShooterSpeed(value from vision)
        public void setShooterSpeed(double shooter_speed_in) {
            m_shooterSpeed = shooter_speed_in;
        }
        public double getShooterSpeed() {
            return m_shooterSpeed;
        }
    }

    public static enum SequenceState {
        None,
        Initiated,
        Firing
    };

    public static enum LEDValue {
        // Color 1 -> Yellow
        // Color 2 -> Green

        Idle(0.69), // solid yellow
        SolidGreen(0.73),
        Color1Flash(0.05), // Heartbeat Medium
        Color2Flash(0.25); // Heartbeat Medium

        public final double value;

        private LEDValue(double value_in) {
            value = value_in;
        }
    }

    private static ShootMode shootMode = ShootMode.Speaker;
    private static SequenceState sequenceState = SequenceState.None;

    public static ShootMode getShootMode() {
        return shootMode;
    }
    private static void setShootMode(ShootMode newShootMode) {
        if (shootMode == newShootMode)
            return;

        sequenceReset();
        shootMode = newShootMode;
    }
    public static final InstantCommand setShootModeToAmpCommand =
        new InstantCommand(() -> setShootMode(ShootMode.Amp));
    public static final InstantCommand setShootModeToClimbTrapCommand =
        new InstantCommand(() -> setShootMode(ShootMode.Climb_Trap));
    public static final InstantCommand setShootModeToSpeakerCommand =
        new InstantCommand(() -> setShootMode(ShootMode.Speaker));

    public static final InstantCommand initiateSequenceCommand =
        new InstantCommand(() -> sequenceInitiate());

    public static SequenceState getSequenceState() {
        return sequenceState;
    }
    public static void sequenceReset() {
        sequenceState = SequenceState.None;
    }
    public static void sequenceInitiate() {
        sequenceState = SequenceState.Initiated;
    }
    public static void sequenceFire() {
        sequenceState = SequenceState.Firing;
    }
}