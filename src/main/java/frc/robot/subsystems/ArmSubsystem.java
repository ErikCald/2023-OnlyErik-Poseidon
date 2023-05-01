// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.custompathplanner.PPArmCommand;
import frc.robot.Config.Arm;
import frc.robot.Config.Arm.ArmFeedforward;
import frc.robot.Config.Arm.ArmPathPlanner;
import frc.robot.Config.Arm.ArmPid;
import frc.robot.Config.Arm.ArmSimulation;
import frc.robot.Config.CANID;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
    public enum Arm1Bias {
        PositiveVelocityBias(0.06, 0.3),
        NegativeVelocityBias(0.06, -0.3),
        HeavyPositiveVelocityBias(0.1, 0.7),
        HeavyNegativeVelocityBias(0.1, -0.7),
        NoBias(0, 0);

        private static final double HEAVY_DETERMINANT_RANGE = 0.1;
        private static final double LIGHT_DETERMINANT_RANGE = 0.06;
        private static final double HEAVY_VELOCITY_BIAS = 0.3;
        private static final double LIGHT_VELOCITY_BIAS = 0.7;

        private double determinantRange, angularVelocityBias;
        private Arm1Bias(double determinantRange, double angularVelocityBias) {
            this.determinantRange = determinantRange;
            this.angularVelocityBias = angularVelocityBias;
        }

        public double getDeterminantRange() {
            return determinantRange;
        }

        public double getAngularVelocityBias() {
            return angularVelocityBias;
        }
    }
    
    ArmDisplay m_armDisplay;

    CANSparkMax m_motor0, m_motor1;
    RelativeEncoder m_neoEncoder0, m_neoEncoder1;
    SparkMaxPIDController m_sparkPid0, m_sparkPid1;

    private static boolean m_haveCone = false;
    private Arm1Bias m_arm1VelBias = Arm1Bias.NoBias;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        m_armDisplay = new ArmDisplay(Units.metersToInches(Arm.ARM0_LENGTH), Units.metersToInches(Arm.ARM1_LENGTH));
        PPArmCommand.setLoggingCallbacks(null, this::setSetpointDisplay, null, null);

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

        // PID
        m_sparkPid0.setFF(ArmPid.ARM0_FF);
        m_sparkPid0.setP(ArmPid.ARM0_PID.kP);
        m_sparkPid0.setI(ArmPid.ARM0_PID.kI);
        m_sparkPid0.setD(ArmPid.ARM0_PID.kD);
        m_sparkPid0.setIZone(ArmPid.ARM0_IZONE);

        m_sparkPid1.setFF(ArmPid.ARM1_FF);
        m_sparkPid1.setP(ArmPid.ARM1_PID.kP);
        m_sparkPid1.setI(ArmPid.ARM1_PID.kI);
        m_sparkPid1.setD(ArmPid.ARM1_PID.kD);
        m_sparkPid1.setIZone(ArmPid.ARM1_IZONE);
        

        REVPhysicsSim.getInstance().addSparkMax(m_motor0, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_motor1, DCMotor.getNEO(1));

        // if (Robot.isSimulation()) {
        //     ArmSimulation.ARM0_SIM.setInput(0);
        //     ArmSimulation.ARM0_SIM.update(0.020);

        //     ArmSimulation.ARM1_SIM.setInitalState(ArmSimulation.ARM0_SIM.getAngleRads(), Math.toRadians(20));
        // }
    }

    @Override
    public void periodic() {
        double neoEncoder0Position = m_neoEncoder0.getPosition();
        double neoEncoder1Position = m_neoEncoder1.getPosition();

        m_armDisplay.updateMeasurementDisplay(neoEncoder0Position, neoEncoder1Position);
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();

        // Get motor duty cycle set during normal operations
        double motor0AppliedOutput = m_motor0.getAppliedOutput();
        double motor1AppliedOutput = m_motor1.getAppliedOutput();

        // System.out.println(motor0AppliedOutput);

        // Disable the simulation when the robot is disabled
        if (DriverStation.isDisabled()) {
            motor0AppliedOutput = 0;
            motor1AppliedOutput = 0;
        }

        System.out.println(RobotController.getBatteryVoltage());

        // Pass the simulation with the inputs as voltages
        ArmSimulation.ARM0_SIM.setInputVoltage(motor0AppliedOutput * RobotController.getBatteryVoltage(), ArmSimulation.ARM1_SIM.getAngleRads(), m_haveCone);
        ArmSimulation.ARM1_SIM.setInputVoltage(motor1AppliedOutput * RobotController.getBatteryVoltage(), ArmSimulation.ARM0_SIM.getAngleRads());

        // Update simulation
        ArmSimulation.ARM0_SIM.update(0.020); // 20ms clock cycle
        ArmSimulation.ARM1_SIM.update(0.020);

        // Update encoder readings
        m_neoEncoder0.setPosition(ArmSimulation.ARM0_SIM.getAngleRads());
        m_neoEncoder1.setPosition(ArmSimulation.ARM1_SIM.getAngleRads());

        // Update the simulation of the load on the battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            ArmSimulation.ARM0_SIM.getCurrentDrawAmps(),
            ArmSimulation.ARM1_SIM.getCurrentDrawAmps()));
    }

    public void setSetpointDisplay(Pose2d targetPose) {
        Rotation2d[] angles = inverseKinematics(targetPose.getTranslation());
        m_armDisplay.updateSetpointDisplay(angles[0].getRadians(), angles[1].getRadians());
    }

    public Rotation2d getArm0Position() {
        return new Rotation2d(m_neoEncoder0.getPosition());
    }

    public Rotation2d getArm1Position() {
        return new Rotation2d(m_neoEncoder1.getPosition());
    }

    private double calculateArm1GravityComp() {
        double enc2AtHorizontal = getArm1Position().getRadians() - (Math.PI - getArm0Position().getRadians());
        double voltsAtHorizontal;
        if (m_haveCone) {
            voltsAtHorizontal = ArmFeedforward.ARM1_HORIZONTAL_VOLTAGE_CONE;
        } else {
            voltsAtHorizontal = ArmFeedforward.ARM1_HORIZONTAL_VOLTAGE;
        }
        return voltsAtHorizontal * Math.cos(enc2AtHorizontal);
    }

    private double calculateArm0GravityComp() {
        return calculateFirstJointMomentDueToGravity(
                getArm0Position().getRadians(), 
                getArm1Position().getRadians()
            ) * ArmFeedforward.ARM0_MOMENT_TO_VOLTAGE;
    }

    public static double calculateFirstJointMomentDueToGravity(double firstJointAngleRad, double secondJointAngleRad) {
        double secondJointAtHorizontal = secondJointAngleRad - (Math.PI - firstJointAngleRad);

        double firstJointMoment = ArmFeedforward.ARM0_FORCE * (ArmFeedforward.LENGTH_ARM0_TO_COG * Math.cos(firstJointAngleRad));
        double secondJointMoment = ArmFeedforward.ARM1_FORCE * (Units.metersToInches(Arm.ARM0_LENGTH) * Math.cos(firstJointAngleRad)
                + ArmFeedforward.LENGTH_ARM1_TO_COG * Math.cos(secondJointAtHorizontal));
        if (m_haveCone == false) {
            return firstJointMoment + secondJointMoment;
        } else {
            double coneMoment = ArmFeedforward.CONE_FORCE
                    * (Units.metersToInches(Arm.ARM0_LENGTH) * Math.cos(firstJointAngleRad) + Units.metersToInches(Arm.ARM1_LENGTH) * Math.cos(secondJointAtHorizontal));
            return firstJointMoment + secondJointMoment + coneMoment;
        }
    }

    public static double calculateMomentOfInertia(double firstJointAngleRad, double secondJointAngleRad) {
        if (m_haveCone) {
            return ArmSimulation.ARM0_MASS_KG * ArmFeedforward.LENGTH_ARM0_TO_COG * ArmFeedforward.LENGTH_ARM0_TO_COG + 
                    ArmSimulation.ARM1_MASS_KG * ArmFeedforward.LENGTH_ARM1_TO_COG * ArmFeedforward.LENGTH_ARM1_TO_COG + 
                    ArmSimulation.CONE_MASS_KG * Arm.ARM1_LENGTH * Arm.ARM1_LENGTH;
        } else {
            return ArmSimulation.ARM0_MASS_KG * ArmFeedforward.LENGTH_ARM0_TO_COG * ArmFeedforward.LENGTH_ARM0_TO_COG + 
                    ArmSimulation.ARM1_MASS_KG * ArmFeedforward.LENGTH_ARM1_TO_COG * ArmFeedforward.LENGTH_ARM1_TO_COG;
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
    public Translation2d forwardKinematics(Rotation2d arm0Angle, Rotation2d arm1Angle) {
        return new Translation2d(
                    Arm.ARM0_LENGTH, 
                    arm0Angle)
                .plus(new Translation2d(
                    Arm.ARM1_LENGTH, 
                    arm1Angle.minus(new Rotation2d(Math.PI).minus(arm0Angle)))
                );
    }

    public Pose2d forwardKinematicsPose() {
        return new Pose2d(
            forwardKinematics(
                getArm0Position(), 
                getArm1Position()
            ).plus(ArmPathPlanner.IMAGE_TO_ARM_OFFSET),
            new Rotation2d());
    }

    public double[] inverseVelocityKinematics(double xSpeed, double ySpeed) {
        SimpleMatrix velMatrix = new SimpleMatrix(2, 1);
        velMatrix.setColumn(0, 0, xSpeed, ySpeed);

        double a0 = Arm.ARM0_LENGTH;
        double a1 = Arm.ARM1_LENGTH;
        double q0 = getArm0Position().getRadians();
        double q1 = getArm1Position().getRadians() - Math.PI;
        double q0q1 = q0+q1;


        // ATTEMPT 1
        // double inverseScalar = 1 / (a0*a1*Math.sin(q1));
        // SimpleMatrix jacobian = new SimpleMatrix(2, 2);
        // jacobian.setRow(0, 0, 
        //     inverseScalar*a1*Math.cos(q0q1),
        //     inverseScalar*a1*Math.sin(q0q1)
        // );
        // jacobian.setRow(1, 0, 
        //     inverseScalar*-a0*Math.cos(q0)-a1*Math.cos(q0q1),
        //     inverseScalar*-a0*Math.sin(q0)-a1*Math.sin(q0q1)
        // );


        // ATTEMPT 2
        SimpleMatrix jacobian = new SimpleMatrix(2, 2);
        jacobian.setRow(0, 0, 
            -a0*Math.sin(q0)-a1*Math.sin(q0q1),
            -a1*Math.sin(q0q1)
        );
        jacobian.setRow(1, 0, 
            a0*Math.cos(q0)+a1*Math.cos(q0q1),
            a1*Math.cos(q0q1)
        );
        double determinant = jacobian.determinant();
        jacobian = jacobian.invert();

        
        SimpleMatrix angularVelsMatrix = jacobian.mult(velMatrix); 

        return new double[]{angularVelsMatrix.get(0, 0), angularVelsMatrix.get(1, 0), determinant};
    }

    public void setArmSpeeds(ChassisSpeeds speeds) {
        double[] angularSpeeds = inverseVelocityKinematics(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        if (m_arm1VelBias == Arm1Bias.PositiveVelocityBias) {
            if (Math.abs(angularSpeeds[2]) < m_arm1VelBias.getDeterminantRange() ||
                    getArm1Position().getDegrees() <= 179) {
                setArm0Speed(0);
                setArm1Speed(2);
            } else {
                if (Math.abs(angularSpeeds[2]) < 0.04) {
                    setArm0Speed(0);
                } else {
                    setArm0Speed(angularSpeeds[0]);
                }
                setArm1Speed(angularSpeeds[1]);
            }
        }
        else if (m_arm1VelBias == Arm1Bias.NegativeVelocityBias) {
            if (Math.abs(angularSpeeds[2]) < m_arm1VelBias.getDeterminantRange() ||
                    getArm1Position().getDegrees() >= 179) {
                setArm0Speed(0);
                setArm1Speed(-2);
            } else {
                if (Math.abs(angularSpeeds[2]) < 0.04) {
                    setArm0Speed(0);
                } else {
                    setArm0Speed(angularSpeeds[0]);
                }
                setArm1Speed(angularSpeeds[1]);
            }
        }
        else if (m_arm1VelBias == Arm1Bias.HeavyPositiveVelocityBias) {
            setArm0Speed(0);

            if (Math.abs(angularSpeeds[2]) < m_arm1VelBias.getDeterminantRange() ||
                    getArm1Position().getDegrees() <= 180+15) {
                
                setArm1Speed(7);
            } else {
                setArm1Speed(0);
            }
        }
        else if (m_arm1VelBias == Arm1Bias.HeavyNegativeVelocityBias) {
            setArm0Speed(0);

            if (Math.abs(angularSpeeds[2]) < m_arm1VelBias.getDeterminantRange() ||
                    getArm1Position().getDegrees() >= 180-15) {
                
                setArm1Speed(-7);
            } else {
                setArm1Speed(0);
            }
        }
        else if (m_arm1VelBias == Arm1Bias.NoBias) {
            if (Math.abs(angularSpeeds[2]) > 0.04) {
                setArm0Speed(angularSpeeds[0]);
                setArm1Speed(angularSpeeds[1]);
            } else {
                setArm0Speed(0);
                setArm1Speed(0);
            }
        }        
    }

    public void setArm0Speed(double angularSpeed) {
        double voltage;
        if (!Robot.isSimulation() || ArmSimulation.SIMULATE_GRAVITY)  {
            voltage = ArmFeedforward.ARM0_SIMEPLE_FF.calculate(angularSpeed) + calculateArm0GravityComp();
        } else {
            voltage = ArmFeedforward.ARM0_SIMEPLE_FF.calculate(angularSpeed);
        }
        // System.out.println(angularSpeed +", "+ voltage);

        // m_sparkPid0.setReference(angularSpeed, ControlType.kVelocity, 0, voltage);

        m_sparkPid0.setReference(voltage, ControlType.kVoltage);
    }

    public void setArm1Speed(double angularSpeed) {
        double voltage;
        if (!Robot.isSimulation() || ArmSimulation.SIMULATE_GRAVITY)  {
            voltage = ArmFeedforward.ARM1_SIMEPLE_FF.calculate(angularSpeed) + calculateArm1GravityComp();
        } else {
            voltage = ArmFeedforward.ARM1_SIMEPLE_FF.calculate(angularSpeed);
        }
        
        // m_sparkPid1.setReference(angularSpeed, ControlType.kVelocity, 0, voltage);

        m_sparkPid1.setReference(voltage, ControlType.kVoltage);
    }

    public CommandBase setArm1VelocityBias(Arm1Bias bias) {
        return Commands.runOnce(() -> m_arm1VelBias = bias);
    }
}