package frc.lib.custompathplanner;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PPArmCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final PPArmController controller;
    private final Consumer<ChassisSpeeds> outputArmSpeeds;

    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
    private static Consumer<Pose2d> logTargetPose = null;
    private static Consumer<ChassisSpeeds> logSetpoint = null;
    private static BiConsumer<Translation2d, Rotation2d> logError = PPArmCommand::defaultLogError;

    public PPArmCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<ChassisSpeeds> outputArmSpeeds,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.controller = new PPArmController(xController, yController, rotationController);
        this.outputArmSpeeds = outputArmSpeeds;

        addRequirements(requirements);

    }

    @Override
    public void initialize() {
        if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(trajectory);
        }

        timer.reset();
        timer.start();

        PathPlannerServer.sendActivePath(trajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

        Pose2d currentPose = this.poseSupplier.get();

        PathPlannerServer.sendPathFollowingData(
                new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
                currentPose);

        ChassisSpeeds targetArmSpeeds = this.controller.calculate(currentPose, desiredState);

        this.outputArmSpeeds.accept(targetArmSpeeds);

        if (logTargetPose != null) {
            logTargetPose.accept(
                    new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
        }

        if (logError != null) {
            logError.accept(
                    currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
                    currentPose.getRotation().minus(desiredState.holonomicRotation));
        }

        if (logSetpoint != null) {
            logSetpoint.accept(targetArmSpeeds);
        }

        // System.out.println(currentPose);
        // System.out.println(targetArmSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();

        if (interrupted|| Math.abs(this.trajectory.getEndState().velocityMetersPerSecond) < 0.1) {
            this.outputArmSpeeds.accept(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
    }

    private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber("PPArmCommand/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPArmCommand/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber("PPArmCommand/rotationErrorDegrees", rotationError.getDegrees());
    }

    /**
     * Set custom logging callbacks for this command to use instead of the default
     * configuration of
     * pushing values to SmartDashboard
     *
     * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory
     *                            representing the
     *                            active path. This will be called whenever a
     *                            PPRamseteCommand starts
     * @param logTargetPose       Consumer that accepts a Pose2d representing the
     *                            target pose while path
     *                            following
     * @param logSetpoint         Consumer that accepts a ChassisSpeeds object
     *                            representing the setpoint
     *                            speeds
     * @param logError            BiConsumer that accepts a Translation2d and
     *                            Rotation2d representing the error
     *                            while path following
     */
    public static void setLoggingCallbacks(
            Consumer<PathPlannerTrajectory> logActiveTrajectory,
            Consumer<Pose2d> logTargetPose,
            Consumer<ChassisSpeeds> logSetpoint,
            BiConsumer<Translation2d, Rotation2d> logError) {
        PPArmCommand.logActiveTrajectory = logActiveTrajectory;
        PPArmCommand.logTargetPose = logTargetPose;
        PPArmCommand.logSetpoint = logSetpoint;
        // PPArmCommand.logError = logError;
    }
}