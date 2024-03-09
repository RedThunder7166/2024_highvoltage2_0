package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    //private TalonFX mAngleMotor;
    // private VictorSPX mAngleMotor;
    private TalonSRX mAngleMotor;
    private TalonFX mDriveMotor;
    // private CANCoder angleEncoder;

    private PIDController angleController = new PIDController(
        Constants.Swerve.angleKP,
        moduleNumber, moduleNumber);

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    // private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        // angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonSRX(moduleConstants.angleMotorID);
        final TalonSRXConfiguration config = Robot.ctreConfigs.swerveAngleSRXConfigs[moduleNumber];
        mAngleMotor.configAllSettings(config);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition); // this is done in ctreconfigs;
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public double getTurnSelectedSensorPosition() {
        return mAngleMotor.getSensorCollection().getPulseWidthPosition();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        SmartDashboard.putNumber("Module " + moduleNumber + " Desired State angle", desiredState.angle.getDegrees());
        mAngleMotor.set(TalonSRXControlMode.Position, Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.Swerve.angleGearRatio));
        // mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSensorCollection().getPulseWidthPosition(), Constants.Swerve.angleGearRatio));
    }

    // public Rotation2d getCANcoder(){
    //     return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    // }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio) - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor.setPosition(absolutePosition);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            getAngle()
        );
    }
}