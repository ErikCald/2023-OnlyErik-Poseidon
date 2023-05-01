// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class TestArmDynamics extends CommandBase {
    private ArmSubsystem m_subsystem;
    private CommandXboxController m_controller;
    /** Creates a new TestArmDynamics. */
    public TestArmDynamics(ArmSubsystem armSubsystem, CommandXboxController controller) {
        m_subsystem = armSubsystem;
        m_controller = controller;

        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(m_controller.getRawAxis(0), 0.2) * 3;
        double ySpeed = -1 * MathUtil.applyDeadband(m_controller.getRawAxis(1), 0.2) * 3;

        m_subsystem.setArmSpeeds(new ChassisSpeeds(xSpeed, ySpeed, 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
