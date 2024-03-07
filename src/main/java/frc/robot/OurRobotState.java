// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public final class OurRobotState {
    public static enum ShootMode {
        // TODO: get ShootMode aim positions and shooter speeds
        Amp(Constants.Aim.COLLISION_AVOIDANCE_POSITION, 0),
        Climb_Trap(Constants.Aim.COLLISION_AVOIDANCE_POSITION, 0),
        Speaker(0 /* this should never be used */, 0, true);

        public final double m_position;
        private double m_shooterSpeed;
        public final boolean m_autoAim;

        private ShootMode(double position, double shooter_speed, boolean auto_aim) {
            m_position = position;
            m_shooterSpeed = shooter_speed;
            m_autoAim = auto_aim;
        }
        private ShootMode(double position, double shooter_speed) {
            this(position, shooter_speed, false);
        }

        // TODO: ShootMode.Speaker.setShooterSpeed(value from vision)
        public void setShooterSpeed(double shooter_speed_in) {
            m_shooterSpeed = shooter_speed_in;
        }
        public double getShooterSpeed() {
            return m_shooterSpeed;
        }
    }

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

    private static boolean intakeEntranceSensorState = false;
    private static boolean intakeExitSensorState = false;

    public static ShootMode getShootMode() {
        return shootMode;
    }
    public static void setShootMode(ShootMode newShootMode) {
        shootMode = newShootMode;
    }
    public static final InstantCommand setShootModeToAmpCommand = new InstantCommand(() -> {
        shootMode = ShootMode.Amp;
    });
    public static final InstantCommand setShootModeToClimbTrapCommand = new InstantCommand(() -> {
        shootMode = ShootMode.Climb_Trap;
    });
    public static final InstantCommand setShootModeToSpeakerCommand = new InstantCommand(() -> {
        shootMode = ShootMode.Speaker;
    });
    public static final InstantCommand initiateCurrentShootModeCommand = new InstantCommand(() -> {
        // switch (shootMode) {
        //     case Amp:
                
        //         break;

        //     case Climb_Trap:

        //         break;

        //     case Speaker:

        //         break;

        //     default:
        //         break;
        // }

        
    });

    // TODO: this might not be neccessary. if not, remove it
    public static void periodic() {
        
    }
}
