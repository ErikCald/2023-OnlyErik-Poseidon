// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The Config class provides a convenient place for robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 */
public final class Config {
    public static class JoystickConfig {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int OPERATOR_JOYSTICK_PORT = 1;
    }

    public static class Arm {

        public static final double ARM0_LENGTH = 27.75; // inches
        public static final double ARM1_LENGTH = 38.6; // inches

        public static final double ARM0_GEAR_RATIO = 62.5; // A number greater than 1 represents a reduction
        public static final double ARM1_GEAR_RATIO = 23.5;

        public static final boolean ARM0_INVERTED = true;
        public static final boolean ARM1_INVERTED = true;

        public static final int ARM0_CURRENT_LIMIT = 60;
        public static final int ARM1_CURRENT_LIMIT = 60;

        public static final float ARM0_FORWARD_LIMIT = (float) Math.toRadians(135); // floats for CANSparkMax API
        public static final float ARM0_REVERSE_LIMIT = (float) Math.toRadians(40);
        public static final float ARM1_FORWARD_LIMIT = (float) Math.toRadians(190);
        public static final float ARM1_REVERSE_LIMIT = (float) Math.toRadians(7);
        public static final boolean ARM0_SOFT_LIMIT_ENABLE = true;
        public static final boolean ARM1_SOFT_LIMIT_ENABLE = true;

        

        public static class ArmFeedforward {
            public static final double TOP_HORIZONTAL_VOLTAGE = 1.5;
            public static final double TOP_HORIZONTAL_VOLTAGE_CONE = 2.3;
            public static final double BOTTOM_MOMENT_TO_VOLTAGE = 0.000005;

            public static final double LENGTH_BOTTOM_ARM_TO_COG = 14.56;
            public static final double LENGTH_TOP_ARM_TO_COG = 28.22;

            public static final double GRAVITATIONAL_CONSTANT = 389.0886; // inches/s/s  which is equal to 9.81 m/s/s
            public static final double BOTTOM_ARM_FORCE = 11.29 * GRAVITATIONAL_CONSTANT; // 11.29 lb
            public static final double TOP_ARM_FORCE = 7.77 * GRAVITATIONAL_CONSTANT; // 7.77 lb
            public static final double CONE_ARM_FORCE = 1.21 * GRAVITATIONAL_CONSTANT; // 1.21 lb
        }        

        public static class ArmSimulation {
            public static final double ARM0_MASS = 0;
            public static final double ARM1_MASS = 0;

            public static final double ARM0_NOISE = 2.0 * Math.PI / 4096;
            public static final double ARM1_NOISE = 2.0 * Math.PI / 4096;

            public static final SingleJointedArmSim ARM0_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM0_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM0_LENGTH, ARM0_MASS),
                    ARM0_LENGTH,
                    Units.degreesToRadians(-75),
                    Units.degreesToRadians(255),
                    false, // Don't simulate gravity on the bottom arm
                    VecBuilder.fill(ARM0_NOISE) // Add noise with a small std-dev
            );

            public static final SingleJointedArmSim ARM1_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM1_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM1_LENGTH, ARM1_MASS),
                    ARM1_LENGTH,
                    Units.degreesToRadians(-75),
                    Units.degreesToRadians(255),
                    true,
                    VecBuilder.fill(ARM1_NOISE) // Add noise with a small std-dev
            );
        }
    }

}
