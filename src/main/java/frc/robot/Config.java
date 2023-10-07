// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.lib3512.config.CTREConfigs;
import frc.lib.lib3512.config.SwerveModuleConstants;

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

        public static final int TRANSLATION_AXIS = XboxController.Axis.kLeftY.value;
        public static final int STRAFE_AXIS = XboxController.Axis.kLeftX.value;
        public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;
    }

    /**
     * CAN IDs, ports, channels, etc.
     */
    public static class CANID {
        public static int PIGEON = 30;
        public static int CANDLE = 15;// robotSpecific(15, 15, -1, 15, 15);
        public static int CTRE_PCM = 1;// robotSpecific(1, 1, -1, -1);

        // Swerve Drive
        public static final int MOD0_DRIVE = 24;
        public static final int MOD0_STEER = 23;
        public static final int MOD0_CANCODER = 9;

        public static final int MOD1_DRIVE = 21;
        public static final int MOD1_STEER = 25;
        public static final int MOD1_CANCODER = 6;

        public static final int MOD2_DRIVE = 20;
        public static final int MOD2_STEER = 26;
        public static final int MOD2_CANCODER = 8;

        public static final int MOD3_DRIVE = 27;
        public static final int MOD3_STEER = 22;
        public static final int MOD3_CANCODER = 7;

        // Arm Subsystem
        public static final int ARM1_SPARK = 5;
        public static final int ARM0_SPARK = 4;
    }

    public static int ANALOG_SELECTOR_PORT = 0;

    public static final class Swerve {
        public static final double JOYSTICK_DEADBAND = 0.1;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.52;
        public static final double WHEEL_BASE = 0.655;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double MK4_L1_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
        public static final double DRIVE_GEAR_RATIO = MK4_L1_GEAR_RATIO; // 8.14:1
        public static final double STEER_GEAR_RATIO = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // FL
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // FR
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // RL
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)); // RR

        /* Create Cancoder Configs */
        public static final CTREConfigs ctreConfigs = new CTREConfigs();

        /* Swerve Voltage Compensation */
        public static final double VOLTAGE_COMPENSATION = 12.0;

        /* Swerve Current Limiting */
        public static final int STEER_CURRENT_LIMIT = 20;
        public static final int DRIVE_CURRENT_LIMIT = 80;

        /* Steer Motor PID Values */
        public static final double STEER_P = 0.01;
        public static final double STEER_I = 0.0;
        public static final double STEER_D = 0.0;
        public static final double STEER_FF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.1;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_FF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_S = 0.667;
        public static final double DRIVE_V = 2.44;
        public static final double DRIVE_A = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_POS_CONVERSION = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VEL_CONVERSION = DRIVE_POS_CONVERSION / 60.0;
        public static final double STEER_POS_CONVERSION = 360.0 / STEER_GEAR_RATIO;

        /* Swerve max attainable speeds */
        public static final double MAX_ATTAINABLE_SPEED = 3; // meters per second
        // public static final double MAX_ATTAINABLE_ANGULAR_SPEED = Math.PI*3.0; //
        // radians per second

        /* Swerve Teleop speeds */
        public static final double TELEOP_SPEED = 1.0; // meters per second
        public static final double TELEOP_ANGULAR_SPEED = Math.PI; // radians per second

        /* Neutral Modes */
        public static final IdleMode STEER_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERT = false;
        public static final boolean STEER_INVERT = false;

        /* Steer Encoder Invert */
        public static final boolean CANCODER_INVERT = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = CANID.MOD0_DRIVE;
            public static final int STEER_MOTOR_ID = CANID.MOD0_STEER;
            public static final int CANCODER_ID = CANID.MOD0_CANCODER;
            public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(270.73);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    STEER_MOTOR_ID,
                    CANCODER_ID, STEER_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = CANID.MOD1_DRIVE;
            public static final int STEER_MOTOR_ID = CANID.MOD1_STEER;
            public static final int CANCODER_ID = CANID.MOD1_CANCODER;
            public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(159.3);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    STEER_MOTOR_ID,
                    CANCODER_ID, STEER_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = CANID.MOD2_DRIVE;
            public static final int STEER_MOTOR_ID = CANID.MOD2_STEER;
            public static final int CANCODER_ID = CANID.MOD2_CANCODER;
            public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(194.9);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    STEER_MOTOR_ID,
                    CANCODER_ID, STEER_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = CANID.MOD3_DRIVE;
            public static final int STEER_MOTOR_ID = CANID.MOD3_STEER;
            public static final int CANCODER_ID = CANID.MOD3_CANCODER;
            public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(8.5);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    STEER_MOTOR_ID,
                    CANCODER_ID, STEER_OFFSET);
        }
    }

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
        public static final float ARM1_FORWARD_LIMIT = (float) Math.toRadians(320);
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

            public static final double GRAVITATIONAL_CONSTANT = 389.0886; // inches/s/s which is equal to 9.81 m/s/s
            public static final double ARM0_FORCE = 11.29 * GRAVITATIONAL_CONSTANT; // 11.29 lb
            public static final double ARM1_FORCE = 7.77 * GRAVITATIONAL_CONSTANT; // 7.77 lb
            public static final double CONE_FORCE = 1.21 * GRAVITATIONAL_CONSTANT; // 1.21 lb
        }

        public static class ArmSimulation {
            public static final boolean SIMULATE_GRAVITY = false; // SingleJointedArmSim doesn't know it's a double
                                                                  // jointed arm.
            public static final double ARM0_MASS_KG = Units
                    .lbsToKilograms(ArmFeedforward.ARM0_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);
            public static final double ARM1_MASS_KG = Units
                    .lbsToKilograms(ArmFeedforward.ARM1_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);

            public static final double ARM0_NOISE = 2.0 * Math.PI / 4096;
            public static final double ARM1_NOISE = 2.0 * Math.PI / 4096;

            public static final SingleJointedArmSim ARM0_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM0_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM0_LENGTH, ARM0_MASS_KG),
                    ARM0_LENGTH,
                    Arm.ARM0_REVERSE_LIMIT,
                    Arm.ARM0_FORWARD_LIMIT,
                    SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed
                                      // arm.
                    VecBuilder.fill(ARM0_NOISE) // Add noise with a small std-dev
            );

            public static final SingleJointedArmSim ARM1_SIM = new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    ARM1_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ARM1_LENGTH, ARM1_MASS_KG),
                    ARM1_LENGTH,
                    Arm.ARM1_REVERSE_LIMIT,
                    Arm.ARM1_FORWARD_LIMIT,
                    SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed
                                      // arm.
                    VecBuilder.fill(ARM1_NOISE) // Add noise with a small std-dev
            );
        }

        public static class ArmPathPlanner {
            public static final Translation2d IMAGE_TO_ARM_OFFSET = new Translation2d(1.82, 0.46);

            public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0, 0);
            public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(0, 0, 0);

            // public static final double VEL = 0.6;
            // public static final double ACCEL = 0.6;

            public static final double VEL = 4;
            public static final double ACCEL = 4;
        }
    }

}
