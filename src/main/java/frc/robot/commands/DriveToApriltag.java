// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToApriltag extends CommandBase {
    private final int REQUIRED_SAMPLES = 4;
    private final double CANCEL_AFTER_TIME_WITH_NO_DATA = 2;
    private final int m_tagId;
    private final Pose2d m_offset;
    private final boolean m_isWaypoint;

    private Pose2d m_desiredPose;

    /** 
     * Creates a new WaitForVisionData. 
     * 
     * Must run right after a WaitForVisionCommand from VisionSubsystem. 
     * Will have problems otherwise.
     */
    public DriveToApriltag(int tagId, Pose2d offsetFromTarget, boolean isWaypoint) {
        m_tagId = tagId;
        m_offset = offsetFromTarget;
        m_isWaypoint = isWaypoint;

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SwerveSubsystem.getInstance().resetDriveToPose();
    }

    @Override
    public void execute() {
        if (VisionSubsystem.getInstance().getNumSamples(m_tagId) > REQUIRED_SAMPLES) {
            Pose2d apriltagPose = VisionSubsystem.getInstance().getTargetPose(m_tagId);


            m_desiredPose = new Pose2d(
                apriltagPose.getTranslation().plus(m_offset.getTranslation()),
                apriltagPose.getRotation().plus(m_offset.getRotation()));
            
            if (m_isWaypoint) {
                m_desiredPose = new Pose2d(
                    m_desiredPose.getTranslation(), 
                    VisionSubsystem.getInstance().calcHeadingToFaceApriltag(m_tagId));
            }

            SwerveSubsystem.getInstance().driveToPose(m_desiredPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!m_isWaypoint) {
            SwerveSubsystem.getInstance().stopMotors();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (VisionSubsystem.getInstance().getTimeSinceLastData(m_tagId) > CANCEL_AFTER_TIME_WITH_NO_DATA) {
            SwerveSubsystem.getInstance().stopMotors();
            
            DriverStation.reportWarning(
                String.format(
                    "DriveToApriltag did not get vision data for %.1f seconds while trying to drive to tagId %d",
                    VisionSubsystem.getInstance().getTimeSinceLastData(m_tagId), m_tagId),
                false);

            // Schedule a dummy command with the SwerveSubsystem as a requirement to cancel any CommandGroups this command was in.
            Commands.runOnce(
                () -> {},
                SwerveSubsystem.getInstance()
            ).schedule(); 

            // Cancel itself just to be sure
            this.cancel();

            return false;
        }

        return SwerveSubsystem.getInstance().isAtPose(m_desiredPose, !m_isWaypoint);
    }
}
