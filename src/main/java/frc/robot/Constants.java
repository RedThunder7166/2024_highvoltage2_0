package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 27;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(20.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = Units.inchesToMeters(3 * Math.PI);

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (22 / 14) * 3;
        // public static final double angleGearRatio = (41.25 / 1);
        public static final double angleGearRatio = (1);

        /* Motor Inverts */
        // public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final boolean angleMotorInvert = true;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        // public static final double angleCurrentThresholdTime = 0.1;
        // 0.1 seconds is 100 miliseconds
        public static final int angleCurrentThresholdTime = 100;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.005;
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        // public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AllianceColor {
        public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        public static final boolean is_red_alliance = alliance == Alliance.Red;

        public static final int SOURCE_RIGHT = is_red_alliance ? 9 : 1;
        public static final int SOURCE_LEFT = is_red_alliance ? 10 : 2;

        public static final int SUBWOOFER_SHIFTED = is_red_alliance ? 3 : 8;
        public static final int SUBWOOFER_CENTER = is_red_alliance ? 4 : 7;

        public static final int AMP = is_red_alliance ? 5 : 6;

        // official names that I cannot wrap my head around
        /* from https://www.youtube.com/watch?v=tElgzVLql08 (2024 Field Tour Video: Stage)
            .....................................................
            ...........................@.........................
            ...........................@.........................
            ...........................@.........................
            .........Stage.............@..........Stage..........
            .........Right.............@..........Left...........
            .........11/15.............@..........12/16..........
            ..........................@@@........................
            ........................@@@.@@@......................
            ......................@@@......@@@...................
            ....................@@@...........@@@................
            .................@@@.....Stage......@@@..............
            ...............@@@.......Center.......@@@............
            .............@@@.........13/14..........@@@..........
            .....................................................
        */
        public static final int STAGE_RIGHT = is_red_alliance ? 11 : 15;
        public static final int STAGE_LEFT = is_red_alliance ? 12 : 16; 
        public static final int STAGE_CENTER = is_red_alliance ? 13 : 14;
    }


    public static final class Aim {
        public static final int ANGLE_MOTOR_ID = 14;
        // TODO: find angle cancoder id
        public static final int ANGLE_CANCODER_ID = -1 + 3;

        public static final double MINIMUM_PID_ERROR = 0.5;

        // TODO: get actual angle motor gear ratio
        public static final double ANGLE_MOTOR_GEAR_RATIO = 1 / 1;

        // TODO: find aim collision avoidance position that will be used amp and trap position
        public static final double COLLISION_AVOIDANCE_POSITION = -1;
    }

    public static final class AmpTrap {
        public static final int ROLLER_MOTOR_ID = 10;
        public static final int TOP_SENSOR_ID = 3;

        // TODO: find amp / trap percent output
        public static final double PERCENT_OUTPUT = 0.2;

    }

    public static final class Elevator {
        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;
        // TODO: fact check if elevator right motor opposes left or not
        public static final boolean RIGHT_MOTOR_OPPOSES_LEFT = true;

        public static final double TOP_POSITION = 0;

        public static final int BOTTOM_BUMPER_SENSOR_ID = 4;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 9;
        public static final int INDEXING_MOTOR_ID = 13;

        public static final int ENTRANCE_SENSOR_ID = 1;
        public static final int EXIT_SENSOR_ID = 2;

        // TODO: verify intake current limits
        public static final double INTAKE_MOTOR_CURRENT_LIMIT = 30;

        // TODO: find real intake percent output
        public static final double INTAKE_PERCENT_OUTPUT = 0.2;
        public static final double INDEXING_PERCENT_OUTPUT = 0.2;
    }

    public static final class LED {
        // TODO: get real blinkin id
        public static final int BLINKIN_ID = -1 + 4;
    }

    public static final class Shooter {
        public static final int UPPER_SHOOTER_MOTOR_ID = 15;
        public static final int LOWER_SHOOTER_MOTOR_ID = 16;

        public static final int EXIT_SENSOR_ID = 6;

        public static final boolean LOWER_MOTOR_OPPOSES_UPPER = true;

        public static final double PERCENT_OUTPUT = 0.2;
    }
}
