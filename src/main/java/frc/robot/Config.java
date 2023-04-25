// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
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

    /**
     * CAN IDs, ports, channels, etc.
     */
    public static class CANID {   
        public static int PIGEON = 30; //robotSpecific(30, 27, 27, 27, 30);
    
        public static int CANDLE = 15;//robotSpecific(15, 15, -1, 15, 15);
        public static int CTRE_PCM = 1;//robotSpecific(1, 1, -1, -1);

        // Swerve Drive
        public static final int FRONT_LEFT_DRIVE = 24;
        public static final int REAR_LEFT_DRIVE = 20;
        public static final int FRONT_RIGHT_DRIVE = 21;
        public static final int REAR_RIGHT_DRIVE = 27;

        public static final int FRONT_LEFT_STEERING = 23;
        public static final int REAR_LEFT_STEERING = 26;
        public static final int FRONT_RIGHT_STEERING = 25;
        public static final int REAR_RIGHT_STEERING = 22;

        public static final int FRONT_LEFT_CANCODER = 9;
        public static final int REAR_LEFT_CANCODER = 8;
        public static final int FRONT_RIGHT_CANCODER = 6;
        public static final int REAR_RIGHT_CANCODER = 7;

        // Arm Subsystem
        public static final int ARM1_SPARK = 5;
        public static final int ARM0_SPARK = 4;
    }
    
    public static int ANALOG_SELECTOR_PORT = 0;

    public static class Arm {

        public static final double ARM0_LENGTH = Units.inchesToMeters(27.75);
        public static final double ARM1_LENGTH = Units.inchesToMeters(38.6);

        public static final double ARM0_GEAR_RATIO = 62.5; // A number greater than 1 represents a reduction
        public static final double ARM1_GEAR_RATIO = 60;// 23.5;

        public static final boolean ARM0_INVERTED = true;
        public static final boolean ARM1_INVERTED = true;

        public static final int ARM0_CURRENT_LIMIT = 60;
        public static final int ARM1_CURRENT_LIMIT = 60;

        public static final IdleMode ARM0_IDLEMODE = IdleMode.kBrake;
        public static final IdleMode ARM1_IDLEMODE = IdleMode.kBrake;

        public static final float ARM0_FORWARD_LIMIT = (float) Math.toRadians(200); // floats for CANSparkMax API
        public static final float ARM0_REVERSE_LIMIT = (float) Math.toRadians(25);
        public static final float ARM1_FORWARD_LIMIT = (float) Math.toRadians(190);
        public static final float ARM1_REVERSE_LIMIT = (float) Math.toRadians(7);
        public static final boolean ARM0_SOFT_LIMIT_ENABLE = true;
        public static final boolean ARM1_SOFT_LIMIT_ENABLE = true;

        // Conversion Factors (CF)
        public static final double ARM0_CF_POS = 2 * Math.PI / ARM0_GEAR_RATIO;
        public static final double ARM0_CF_VEL = ARM0_CF_POS / 60.0;
        public static final double ARM1_CF_POS = 2 * Math.PI / ARM1_GEAR_RATIO;
        public static final double ARM1_CF_VEL = ARM1_CF_POS / 60.0;
        
        public static class ArmPid {
            public static final PIDConstants ARM0_PID = new PIDConstants(0.1, 0, 0);
            public static final double ARM0_FF = 0;
            public static final double ARM0_IZONE = 0;

            public static final PIDConstants ARM1_PID = new PIDConstants(0.1, 0, 0);
            public static final double ARM1_FF = 0;
            public static final double ARM1_IZONE = 0;
        }

        public static class ArmFeedforward {
            /**
             * Angular velocity feedforward
             */
            public static final SimpleMotorFeedforward ARM0_SIMEPLE_FF = new SimpleMotorFeedforward(0, 0.11, 0);
            public static final SimpleMotorFeedforward ARM1_SIMEPLE_FF = new SimpleMotorFeedforward(0, 0.11, 0);

            /**
             * Gravity Compensation
             */
            public static final double ARM1_HORIZONTAL_VOLTAGE = 1.5;
            public static final double ARM1_HORIZONTAL_VOLTAGE_CONE = 2.3;
            public static final double ARM0_MOMENT_TO_VOLTAGE = 0.000005; 

            public static final double LENGTH_ARM0_TO_COG = 14.56;
            public static final double LENGTH_ARM1_TO_COG = 28.22;

            public static final double GRAVITATIONAL_CONSTANT = 389.0886; // inches/s/s  which is equal to 9.81 m/s/s
            public static final double ARM0_FORCE = 11.29 * GRAVITATIONAL_CONSTANT; // 11.29 lb
            public static final double ARM1_FORCE = 7.77 * GRAVITATIONAL_CONSTANT; // 7.77 lb
            public static final double CONE_FORCE = 1.21 * GRAVITATIONAL_CONSTANT; // 1.21 lb
        }        

        public static class ArmSimulation {
            public static final boolean SIMULATE_GRAVITY = false; // SingleJointedArmSim doesn't know it's a double jointed arm.
            public static final double ARM0_MASS_KG = Units.lbsToKilograms(ArmFeedforward.ARM0_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);
            public static final double ARM1_MASS_KG = Units.lbsToKilograms(ArmFeedforward.ARM1_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);

            public static final double ARM0_NOISE = 0;// 2.0 * Math.PI / 4096;
            public static final double ARM1_NOISE = 0;// 2.0 * Math.PI / 4096;

            public static final SingleJointedArmSim ARM0_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM0_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM0_LENGTH, ARM0_MASS_KG),
                    ARM0_LENGTH,
                    Arm.ARM0_REVERSE_LIMIT,
                    Arm.ARM0_FORWARD_LIMIT,
                    SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed arm.
                    VecBuilder.fill(ARM0_NOISE) // Add noise with a small std-dev
            );

            public static final SingleJointedArmSim ARM1_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM1_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM1_LENGTH, ARM1_MASS_KG),
                    ARM1_LENGTH, 
                    Arm.ARM1_REVERSE_LIMIT,
                    Arm.ARM1_FORWARD_LIMIT,
                    SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed arm.
                    VecBuilder.fill(ARM1_NOISE) // Add noise with a small std-dev
            );
        }

        public static class ArmPathPlanner {
            public static final Translation2d IMAGE_TO_ARM_OFFSET = new Translation2d(1.82, 0.46);

            public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0, 0);
            public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(0, 0, 0);

            public static final double VEL = 4;
            public static final double ACCEL = 4;
        }
    }

}
