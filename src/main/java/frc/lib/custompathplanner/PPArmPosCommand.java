package frc.custompathplanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class PPArmPosCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final ArmSubsystem m_subsystem;
    
    public PPArmPosCommand(PathPlannerTrajectory trajectory, ArmSubsystem subsystem) {
        m_trajectory = trajectory;
        m_subsystem = subsystem;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        PathPlannerServer.sendActivePath(m_trajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = m_timer.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(currentTime);

        Pose2d currentPose = m_subsystem.forwardKinematicsPose();

        PathPlannerServer.sendPathFollowingData(
                new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
                currentPose);

        m_subsystem.setPoseSetpoint(desiredState.poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
