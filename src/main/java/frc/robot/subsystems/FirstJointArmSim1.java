package frc.robot.subsystems;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Config.Arm;
import frc.robot.Config.Arm.ArmFeedforward;
import frc.robot.Config.Arm.ArmSimulation;

public class FirstJointArmSim1 extends SingleJointedArmSim {

    double currentSecondJointAngleRad = 0;
    boolean hasCone = false;



    // The length of the arm.
    private final double m_armLenMeters;

    // The minimum angle that the arm is capable of.
    private final double m_minAngle;

    // The maximum angle that the arm is capable of.
    private final double m_maxAngle;

    // Whether the simulator should simulate gravity.
    private final boolean m_simulateGravity;


    /**
     * Creates a simulated arm mechanism.
     *
     * @param gearbox            The type of and number of motors in the arm
     *                           gearbox.
     * @param gearing            The gearing of the arm (numbers greater than 1
     *                           represent reductions).
     * @param jKgMetersSquared   The moment of inertia of the arm; can be calculated
     *                           from CAD software.
     * @param armLengthMeters    The length of the arm.
     * @param minAngleRads       The minimum angle that the arm is capable of.
     * @param maxAngleRads       The maximum angle that the arm is capable of.
     * @param simulateGravity    Whether gravity should be simulated or not.
     * @param measurementStdDevs The standard deviations of the measurements.
     */
    public FirstJointArmSim1(
            DCMotor gearbox,
            double gearing,
            double jKgMetersSquared,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            Matrix<N1, N1> measurementStdDevs) {
        super(gearbox,
                gearing,
                jKgMetersSquared,
                armLengthMeters,
                minAngleRads,
                maxAngleRads,
                simulateGravity,
                measurementStdDevs);
        
        m_armLenMeters = armLengthMeters;
        m_minAngle = minAngleRads;
        m_maxAngle = maxAngleRads;
        m_simulateGravity = simulateGravity;
    }

    // public void setInitalState(double firstJointAngleRad, double secondJointAngleRad) {
    //     prevFirstJointAngleRad = firstJointAngleRad;
    //     m_x.set(0, 0, secondJointAngleRad - (Math.PI - firstJointAngleRad));
    // }

    /**
     * Sets the input voltage for the arm.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts, double secondJointAngleRad, boolean hasCone) {
        setInput(volts);
        currentSecondJointAngleRad = secondJointAngleRad;
        this.hasCone = hasCone;
    }

    /**
   * Updates the state of the arm.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // The torque on the arm is given by τ = F⋅r, where F is the force applied by
    // gravity and r the distance from pivot to center of mass. Recall from
    // dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
    // torque on the arm, J is the mass-moment of inertia about the pivot axis,
    // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
    //
    // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/J
    //
    // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
    // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
    // rod rotating about it's end, where L is the overall rod length. The mass
    // distribution is assumed to be uniform. Substitute r=L/2 to find:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
    //   α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
    //   α = 3/2⋅g⋅cos(θ)/L
    //
    // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
    //
    //   f(x, u) = Ax + Bu + [0  α]ᵀ
    //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ

    Matrix<N2, N1> updatedXhat =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                double alphaGrav = -1 * calculateFirstJointMomentDueToGravity(x.get(0, 0), currentSecondJointAngleRad, hasCone) /
                                    calculateMomentOfInertia(x.get(0, 0), currentSecondJointAngleRad, hasCone);
                
                xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collision after updating xhat
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minAngle, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxAngle, 0);
    }
    return updatedXhat;
  }



  public static double calculateFirstJointMomentDueToGravity(double firstJointAngleRad, double secondJointAngleRad, boolean hasCone) {
    double secondJointAtHorizontal = secondJointAngleRad - (Math.PI - firstJointAngleRad);

    double firstJointMoment = ArmFeedforward.ARM0_FORCE * (ArmFeedforward.LENGTH_ARM0_TO_COG * Math.cos(firstJointAngleRad));
    double secondJointMoment = ArmFeedforward.ARM1_FORCE * (Units.metersToInches(Arm.ARM0_LENGTH) * Math.cos(firstJointAngleRad)
            + ArmFeedforward.LENGTH_ARM1_TO_COG * Math.cos(secondJointAtHorizontal));
    if (hasCone == false) {
        return firstJointMoment + secondJointMoment;
    } else {
        double coneMoment = ArmFeedforward.CONE_FORCE
                * (Units.metersToInches(Arm.ARM0_LENGTH) * Math.cos(firstJointAngleRad) + Units.metersToInches(Arm.ARM1_LENGTH) * Math.cos(secondJointAtHorizontal));
        return firstJointMoment + secondJointMoment + coneMoment;
    }
}

public static double calculateMomentOfInertia(double firstJointAngleRad, double secondJointAngleRad, boolean hasCone) {
    if (hasCone) {
        return ArmSimulation.ARM0_MASS_KG * ArmFeedforward.LENGTH_ARM0_TO_COG * ArmFeedforward.LENGTH_ARM0_TO_COG + 
                ArmSimulation.ARM1_MASS_KG * ArmFeedforward.LENGTH_ARM1_TO_COG * ArmFeedforward.LENGTH_ARM1_TO_COG + 
                ArmSimulation.CONE_MASS_KG * Arm.ARM1_LENGTH * Arm.ARM1_LENGTH;
    } else {
        return ArmSimulation.ARM0_MASS_KG * ArmFeedforward.LENGTH_ARM0_TO_COG * ArmFeedforward.LENGTH_ARM0_TO_COG + 
                ArmSimulation.ARM1_MASS_KG * ArmFeedforward.LENGTH_ARM1_TO_COG * ArmFeedforward.LENGTH_ARM1_TO_COG;
    }
}
}
