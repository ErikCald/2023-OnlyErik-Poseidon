// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.Arm.ArmFeedforward;
import frc.robot.Config.Arm.ArmSimulation;
import frc.robot.Config.Arm;
import frc.robot.Config.CANID;

public class ArmSubsystem extends SubsystemBase {
    ArmDisplay m_armDisplay;

    CANSparkMax m_motor0, m_motor1;
    RelativeEncoder m_neoEncoder0, m_neoEncoder1;
    SparkMaxPIDController m_sparkPid0, m_sparkPid1;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        m_armDisplay = new ArmDisplay(Arm.ARM0_LENGTH, Arm.ARM1_LENGTH);

        m_motor0 = new CANSparkMax(CANID.ARM0_SPARK, MotorType.kBrushless);
        m_motor1 = new CANSparkMax(CANID.ARM1_SPARK, MotorType.kBrushless);

        m_motor0.restoreFactoryDefaults();
        m_motor1.restoreFactoryDefaults();

        m_sparkPid0 = m_motor0.getPIDController();
        m_sparkPid1 = m_motor1.getPIDController();

        m_neoEncoder0 = m_motor0.getEncoder();
        m_neoEncoder1 = m_motor1.getEncoder();

        m_motor0.setInverted(Arm.ARM0_INVERTED);
        m_motor1.setInverted(Arm.ARM1_INVERTED);

        m_motor0.setIdleMode(Arm.ARM0_IDLEMODE);
        m_motor1.setIdleMode(Arm.ARM1_IDLEMODE);

        m_motor0.setSmartCurrentLimit(Arm.ARM0_CURRENT_LIMIT);
        m_motor1.setSmartCurrentLimit(Arm.ARM1_CURRENT_LIMIT);

        m_neoEncoder0.setPositionConversionFactor(Arm.ARM0_CF_POS);
        m_neoEncoder0.setVelocityConversionFactor(Arm.ARM0_CF_VEL);
        m_neoEncoder1.setPositionConversionFactor(Arm.ARM1_CF_POS);
        m_neoEncoder1.setVelocityConversionFactor(Arm.ARM1_CF_VEL);

        m_motor0.setSoftLimit(SoftLimitDirection.kForward, Arm.ARM0_FORWARD_LIMIT);
        m_motor0.setSoftLimit(SoftLimitDirection.kReverse, Arm.ARM0_REVERSE_LIMIT);
        m_motor1.setSoftLimit(SoftLimitDirection.kForward, Arm.ARM1_FORWARD_LIMIT);
        m_motor1.setSoftLimit(SoftLimitDirection.kReverse, Arm.ARM1_REVERSE_LIMIT);

        m_motor0.enableSoftLimit(SoftLimitDirection.kForward, Arm.ARM0_SOFT_LIMIT_ENABLE);
        m_motor0.enableSoftLimit(SoftLimitDirection.kReverse, Arm.ARM0_SOFT_LIMIT_ENABLE);
        m_motor1.enableSoftLimit(SoftLimitDirection.kForward, Arm.ARM1_SOFT_LIMIT_ENABLE);
        m_motor1.enableSoftLimit(SoftLimitDirection.kReverse, Arm.ARM1_SOFT_LIMIT_ENABLE);

    }

    @Override
    public void periodic() {
        double neoEncoder0Position = m_neoEncoder0.getPosition();
        double neoEncoder1Position = m_neoEncoder1.getPosition();

        m_armDisplay.updateMeasurementDisplay(neoEncoder0Position, neoEncoder1Position);

        m_motor0.set(0.1);
        m_motor1.set(-0.1);
    }

    @Override
    public void simulationPeriodic() {
        // Get motor duty cycle set during normal operations
        double motor0AppliedOutput = m_motor0.getAppliedOutput();
        double motor1AppliedOutput = m_motor1.getAppliedOutput();

        // Disable the simulation when the robot is disabled
        if (DriverStation.isDisabled()) {
            motor0AppliedOutput = 0;
            motor1AppliedOutput = 0;
        }

        // Pass the simulation with the inputs as voltages
        ArmSimulation.ARM0_SIM.setInput(motor0AppliedOutput * RobotController.getBatteryVoltage());
        ArmSimulation.ARM1_SIM.setInput(motor1AppliedOutput * RobotController.getBatteryVoltage());

        // Update simulation
        ArmSimulation.ARM0_SIM.update(0.020); // 20ms clock cycle
        ArmSimulation.ARM1_SIM.update(0.020);

        // Update encoder readings
        m_neoEncoder0.setPosition(ArmSimulation.ARM0_SIM.getAngleRads());
        m_neoEncoder1.setPosition(ArmSimulation.ARM1_SIM.getAngleRads());

        // Update the simulation of the load on the battery
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(ArmSimulation.ARM0_SIM.getCurrentDrawAmps()));
    }

    public Rotation2d getArm0Position() {
        return new Rotation2d(m_neoEncoder0.getPosition());
    }

    public Rotation2d getArm1Position() {
        return new Rotation2d(m_neoEncoder1.getPosition());
    }

    private double calculateFFTop(boolean haveCone, double topSetpoint) {
        double enc2AtHorizontal = getArm1Position().getRadians() - (Math.PI - getArm0Position().getRadians());
        double voltsAtHorizontal;
        if (haveCone) {
            voltsAtHorizontal = ArmFeedforward.ARM1_HORIZONTAL_VOLTAGE_CONE;
        } else {
            voltsAtHorizontal = ArmFeedforward.ARM1_HORIZONTAL_VOLTAGE;
        }
        return voltsAtHorizontal * Math.cos(enc2AtHorizontal);
    }

    private double calculateFFBottom(double encoder1Rad, double encoder2Rad, boolean haveCone) {
        double enc1AtHorizontal = encoder2Rad - (Math.PI - encoder1Rad);
        double arm0Moment = ArmFeedforward.ARM0_FORCE * (ArmFeedforward.LENGTH_ARM0_TO_COG * Math.cos(encoder1Rad));
        double arm1Moment = ArmFeedforward.ARM1_FORCE * (Arm.ARM0_LENGTH * Math.cos(encoder1Rad)
                + ArmFeedforward.LENGTH_ARM1_TO_COG * Math.cos(enc1AtHorizontal));
        if (haveCone == false) {
            return (arm0Moment + arm1Moment) * ArmFeedforward.ARM0_MOMENT_TO_VOLTAGE;
        } else {
            double coneMoment = ArmFeedforward.CONE_FORCE
                    * (Arm.ARM0_LENGTH * Math.cos(encoder1Rad) + Arm.ARM1_LENGTH * Math.cos(enc1AtHorizontal));
            return (arm0Moment + arm1Moment + coneMoment) * ArmFeedforward.ARM0_MOMENT_TO_VOLTAGE;
        }
    }

    /**
     * Perform inverse kinematics to convert an (X, Z) point to arm0 and arm1
     * angles.
     * 
     * @param point A Translation2d where the y is actually z.
     * @return An array of arm0 and arm1 angles in the order: {arm0Angle, arm1Angle}
     */
    public static Rotation2d[] inverseKinematics(Translation2d point) {
        double x = point.getX();
        double z = point.getY();

        double zx = (Math.pow(x, 2) + Math.pow(z, 2));
        // angle1 --> arm1
        double arm1Angle = Math.acos((Math.pow(Arm.ARM0_LENGTH, 2) + Math.pow(Arm.ARM1_LENGTH, 2) - zx)
                / (2 * Arm.ARM0_LENGTH * Arm.ARM1_LENGTH));

        // angle0 --> arm0
        double arm0Angle = (Math.atan2(z, x)
                + Math.acos((Math.pow(Arm.ARM1_LENGTH, 2) - zx - Math.pow(Arm.ARM0_LENGTH, 2))
                        / (-2 * Math.sqrt(zx) * Arm.ARM0_LENGTH)));

        return new Rotation2d[] { new Rotation2d(arm0Angle), new Rotation2d(arm1Angle) };
    }

    /**
     * INCOMPLETE
     * 
     * @param arm0Angle
     * @param arm1Angle
     * @return
     */
    public static Translation2d forwardKinematics(Rotation2d arm0Angle, Rotation2d arm1Angle) {

        // Do math here...

        return new Translation2d();
    }
}
