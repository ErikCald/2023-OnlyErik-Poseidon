// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.custompathplanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmEnableBrakes extends CommandBase {
    private final Pose2d m_endPose;
    private final ArmSubsystem m_armSub;
    private final ArmPneumaticsSubsystem m_armPneumaticsSub;

    /** Creates a new ArmEnableBrakes. */
    public ArmEnableBrakes(Pose2d endPose, ArmSubsystem arm, ArmPneumaticsSubsystem armPneumatics) {
        m_endPose = endPose;
        m_armSub = arm;
        m_armPneumaticsSub = armPneumatics;
        
        addRequirements(arm, armPneumatics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
