package frc.custompathplanner;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmAutoBuilder extends BaseAutoBuilder {
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final Consumer<ChassisSpeeds> outputArmSpeeds;
    private final Subsystem[] armRequirements;

    /**
     * Create an auto builder that will create command groups that will handle path following and
     * triggering events.
     *
     * <p>This auto builder will use PPArmCommand to follow paths.
     *
     * @param poseSupplier A function that supplies the pose of a point on the arm (usually the end affector)
     *     Create an inverseKinematics equation for your arm.
     * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
     *     commands
     * @param eventMap Map of event marker names to the commands that should run when reaching that
     *     marker.
     * @param driveRequirements The subsystems that the path following commands should require.
     *     Usually just a Arm subsystem.
     */
    public ArmAutoBuilder(
            Supplier<Pose2d> poseSupplier,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            Consumer<ChassisSpeeds> outputArmSpeeds,
            Map<String, Command> eventMap,
            Subsystem... armRequirements) {
        super(poseSupplier, (pose) -> {}, eventMap, DrivetrainType.HOLONOMIC, false);

        this.translationConstants = translationConstants;
        this.rotationConstants = rotationConstants;
        this.outputArmSpeeds = outputArmSpeeds;
        this.armRequirements = armRequirements;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new PPArmCommand(
            trajectory, 
            poseSupplier, 
            pidControllerFromConstants(translationConstants),
            pidControllerFromConstants(translationConstants),
            pidControllerFromConstants(rotationConstants),
            outputArmSpeeds, 
            armRequirements);
    }
}
