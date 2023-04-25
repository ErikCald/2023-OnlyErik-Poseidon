// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.custompathplanner.ArmAutoBuilder;
import frc.robot.Config.Arm.ArmPathPlanner;

/** Add your docs here. */
public class ArmPaths {
    ArmAutoBuilder armBuilder;
    ArmSubsystem armSub;

    PathPlannerTrajectory pickupToTop,
                          topToMiddle,
                          middleToPickup;
    List<PathPlannerTrajectory> testPath1;


    public ArmPaths(ArmSubsystem armSubsystem) {
        armSub = armSubsystem;
        Map<String, Command> eventMap = new HashMap<String, Command>();

        armBuilder = new ArmAutoBuilder(
            armSub::forwardKinematicsPose,
            ArmPathPlanner.TRANSLATION_PID_CONSTANTS, 
            ArmPathPlanner.ROTATION_PID_CONSTANTS, 
            armSub::setArmSpeeds, 
            eventMap, 
            armSub
        );

        buildPaths();
    }

    private void buildPaths() {
        pickupToTop = PathPlanner.loadPath("homeToTop", ArmPathPlanner.VEL, ArmPathPlanner.ACCEL);
        topToMiddle = PathPlanner.loadPath("topToMiddle", ArmPathPlanner.VEL, ArmPathPlanner.ACCEL);
        middleToPickup = PathPlanner.loadPath("middleToHome", ArmPathPlanner.VEL, ArmPathPlanner.ACCEL);
        testPath1 = PathPlanner.loadPathGroup("testPath1", ArmPathPlanner.VEL, ArmPathPlanner.ACCEL);
    }

    public CommandBase pickupToTopCommand() {
        return armBuilder.followPath(pickupToTop);
    }

    public CommandBase topToMiddleCommand() {
        return armBuilder.followPath(topToMiddle);
    }

    public CommandBase middleToPickupCommand() {
        return armBuilder.followPath(middleToPickup);
    }

    public CommandBase testPath1() {
        return armBuilder.followPathGroup(testPath1);
    }



}
