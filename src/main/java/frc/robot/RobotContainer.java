package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Subsystems */
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    // private final AimSubsystem m_aimSubsystem = new AimSubsystem(m_visionSubsystem);
    // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_swerveSubsystem.setDefaultCommand(
            new TeleopSwerveCommand(
                m_swerveSubsystem, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false // this is robot centric, but we don't use it
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));

        /* Operator Buttons */
        // operator.a().onTrue(m_intakeSubsystem.toggleForwardCommand);
        // operator.b().whileTrue(m_intakeSubsystem.backwardOnOffCommand);

        operator.povUp().onTrue(OurRobotState.setShootModeToSpeakerCommand);
        operator.povDown().onTrue(OurRobotState.setShootModeToAmpCommand);
        operator.povLeft().onTrue(OurRobotState.setShootModeToClimbTrapCommand);
        operator.povRight().onTrue(OurRobotState.setShootModeToClimbTrapCommand);

        operator.x().onTrue(OurRobotState.initiateCurrentShootModeCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(m_swerveSubsystem);
    }
}
