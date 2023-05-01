// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Config.JoystickConfig;
import frc.robot.auto.Autos;
import frc.robot.commands.TestArmDynamics;
import frc.robot.commands.TestArmJointControl;
import frc.robot.subsystems.ArmPaths;
import frc.robot.subsystems.ArmSubsystem;

import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

    private final ArmPaths m_armPaths = new ArmPaths(m_armSubsystem);

    // Joysticks
    private final CommandXboxController driver = new CommandXboxController(JoystickConfig.DRIVER_JOYSTICK_PORT);
    private final CommandXboxController operator = new CommandXboxController(JoystickConfig.OPERATOR_JOYSTICK_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        // a,b,x,y
        driver.a().onTrue(m_armPaths.testPath1());
        driver.b().onTrue(new SequentialCommandGroup(
                        m_armPaths.pickupToTopCommand(),
                        m_armPaths.topToBackPickup(),
                        m_armPaths.backPickupToTop(),
                        m_armPaths.topToPickup()
        ));
        driver.x().onTrue(m_armPaths.middleToPickupCommand());

        driver.y().toggleOnTrue(new TestArmDynamics(m_armSubsystem, driver));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {        
        // An example command will be run in autonomous
        return Autos.doNothing();
    }
}
