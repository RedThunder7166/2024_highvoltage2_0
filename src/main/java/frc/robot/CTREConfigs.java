package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    // public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonSRXConfiguration swerveAngleSRXConfig0 = new TalonSRXConfiguration();
    public TalonSRXConfiguration swerveAngleSRXConfig1 = new TalonSRXConfiguration();
    public TalonSRXConfiguration swerveAngleSRXConfig2 = new TalonSRXConfiguration();
    public TalonSRXConfiguration swerveAngleSRXConfig3 = new TalonSRXConfiguration();
    public TalonSRXConfiguration[] swerveAngleSRXConfigs = new TalonSRXConfiguration[] {
        swerveAngleSRXConfig0,
        swerveAngleSRXConfig1,
        swerveAngleSRXConfig2,
        swerveAngleSRXConfig3,
    };

    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    // public CANCoderConfiguration swerveCANcoderConfig = new CANCoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        // swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        // swerveCANcoderConfig.sensorDirection = false;

        // /** Swerve Angle Motor Configurations */
        // /* Motor Inverts and Neutral Mode */
        // swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        // swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        for (TalonSRXConfiguration config : swerveAngleSRXConfigs) {
            config.peakCurrentDuration = Constants.Swerve.angleCurrentThresholdTime;
            config.peakCurrentLimit = Constants.Swerve.angleCurrentLimit;

            config.slot0.kP = Constants.Swerve.angleKP;
            config.slot0.kI = Constants.Swerve.angleKI;
            config.slot0.kD = Constants.Swerve.angleKD;

            config.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute.toFeedbackDevice();

            config.feedbackNotContinuous = true;
        }


        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}