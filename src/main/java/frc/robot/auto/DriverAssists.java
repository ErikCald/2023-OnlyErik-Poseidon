// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToApriltag;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.VisionSubsystem;

public final class DriverAssists {
    private final int NUM_SAMPLES_TO_WAIT = 10;

    /** Example static factory for an autonomous command. */
    public static CommandBase doNothing() {
        return null;
    }

    private DriverAssists() {
        throw new UnsupportedOperationException("DriverAssists is a utility class!");
    }

    public static CommandBase alignToTag(CommandXboxController driver, int tagId, Pose2d waypointOffset, Pose2d finalOffset) {
        return Commands.sequence(
            Commands.deadline(
                VisionSubsystem.getInstance().getWaitForVisionCommand(tagId), 
                new TeleopSwerveCommand(driver)),
            new DriveToApriltag(tagId, waypointOffset, true),
            new DriveToApriltag(tagId, finalOffset, false)
        );
    }
}
