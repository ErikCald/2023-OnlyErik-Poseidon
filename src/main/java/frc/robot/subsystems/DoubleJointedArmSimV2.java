// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class DoubleJointedArmSimV2 {
    private final double GRAVITATIONAL_CONSTANT = 9.81;

    private final FirstJointArmSim m_firstJointSim;
    private final SecondJointArmSim m_secondJointSim;
    private final boolean m_simulateGravity;

    private final double fjArmMassKg;
    private final double fjDistanceToCOGMeters;
    private final double sjMassKg;
    private final double sjDistanceToCOGMeters;
    private final double m_firstJointToSecondJointDistance;
    private final double m_thirdBodyMassKg;
    private final double m_thirdBodyDistanceToCOGMeters; // Distance along second arm


    private boolean m_enableThirdBodyMass;

    


    public DoubleJointedArmSimV2(
            DCMotor firstJointGearbox,
            double firstJointGearing,
            Matrix<N1, N1> firstJointMeasurementStdDevs,
            DCMotor secondJointGearbox,
            double secondJointGearing,
            Matrix<N1, N1> secondJointMeasurementStdDevs,
            
            double firstJointMinAngleRad,
            double firstJointMaxAngleRad,
            double secondJointMinAngleRad,
            double secondJointMaxAngleRad,

            double firstArmMassKg,
            double firstArmDistanceToCOGMeters,
            double secondArmMassKg,
            double secondArmDistanceToCOGMeters,
            double firstJointToSecondJointDistance,
            double thirdBodyMassKg,
            double thirdBodyDistanceToCOGMeters,
            boolean simulateGravity) {
             
        fjArmMassKg = firstArmMassKg;
        fjDistanceToCOGMeters = firstArmDistanceToCOGMeters;
        sjMassKg = secondArmMassKg;
        sjDistanceToCOGMeters = secondArmDistanceToCOGMeters;
        m_firstJointToSecondJointDistance = firstJointToSecondJointDistance;
        m_thirdBodyMassKg = thirdBodyMassKg;
        m_thirdBodyDistanceToCOGMeters = thirdBodyDistanceToCOGMeters;
        
        m_firstJointSim = new FirstJointArmSim(firstJointGearbox, firstJointGearing, firstJointMinAngleRad, firstJointMaxAngleRad, firstJointMeasurementStdDevs);
        m_secondJointSim = new SecondJointArmSim(secondJointGearbox, secondJointGearing, secondJointMinAngleRad, secondJointMaxAngleRad, secondJointMeasurementStdDevs);
        m_simulateGravity = simulateGravity;
    }

    public void setInputVoltages(double firstJointVoltage, double secondJointVoltage, boolean enableThirdBodyMass) {
        m_enableThirdBodyMass = enableThirdBodyMass;
        m_firstJointSim.setInputVoltage(firstJointVoltage);
        m_secondJointSim.setInputVoltage(secondJointVoltage);
    }

    public void update(double dtSeconds) {
        m_firstJointSim.update(dtSeconds);
        m_secondJointSim.update(dtSeconds);
    }

    public double getFirstJointAngleRads() {
        return m_firstJointSim.getAngleRads();
    }

    public double getSecondJointAngleRads() {
        return m_secondJointSim.getAngleRads();
    }

    public double getFirstJointCurrentDrawAmps() {
        return m_firstJointSim.getCurrentDrawAmps();
    }

    public double getSecondJointCurrentDrawAmps() {
        return m_secondJointSim.getCurrentDrawAmps();
    }

    public double getFirstJointVelocityRadPerSec() {
        return m_firstJointSim.getVelocityRadPerSec();
    }

    public double getSecondJointVelocityRadPerSec() {
        return m_secondJointSim.getVelocityRadPerSec();
    }



    public class FirstJointArmSim extends SingleJointedArmSim {
        DCMotor fjGearbox;
        double fjGearing;

        double fjMinAngleRads;
        double fjMaxAngleRads;

        public FirstJointArmSim(
                DCMotor gearbox,
                double gearing,
                double minAngleRads,
                double maxAngleRads,
                Matrix<N1, N1> measurementStdDevs) {
            super(gearbox,
                gearing,
                calculateFJMomentOfInertia(true), // Estimated Moment of inertia
                0, // Length of the arm is not needed since gravity calculation is overriden.
                minAngleRads,
                maxAngleRads,
                m_simulateGravity,
                measurementStdDevs);
            
            fjGearbox = gearbox;
            fjGearing = gearing;
            fjMinAngleRads = minAngleRads;
            fjMaxAngleRads = maxAngleRads;
        }

        private double calculateFJMomentDueToGravity(double firstJointAngleRad, double secondJointAngleRad, boolean enableThirdBodyMass) {
            double secondJointAtHorizontal = secondJointAngleRad - (Math.PI - firstJointAngleRad);

            double firstJointMoment = fjArmMassKg * GRAVITATIONAL_CONSTANT * (fjDistanceToCOGMeters * Math.cos(firstJointAngleRad));
            double secondJointMoment = sjMassKg * GRAVITATIONAL_CONSTANT * 
                            (m_firstJointToSecondJointDistance * Math.cos(firstJointAngleRad) + sjDistanceToCOGMeters * Math.cos(secondJointAtHorizontal));

            if (enableThirdBodyMass) {
                double thirdBodyMoment = m_thirdBodyMassKg * GRAVITATIONAL_CONSTANT * (m_firstJointToSecondJointDistance * Math.cos(firstJointAngleRad) 
                                                 + m_thirdBodyDistanceToCOGMeters * Math.cos(secondJointAtHorizontal));
                return firstJointMoment + secondJointMoment + thirdBodyMoment;
            } else {
                return firstJointMoment + secondJointMoment;
            }
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
            // The angular acceleration due to gravity (aplhaGrav) is  added to the linear system dynamics ẋ=Ax+Bu
            //
            //   f(x, u) = Ax + Bu + [0  α]ᵀ
            //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ

            Matrix<N2, N1> updatedXhat =
                NumericalIntegration.rkdp(
                    (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                        Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));

                        if (m_simulateGravity) {
                            double alphaGrav = -1 * calculateFJMomentDueToGravity(x.get(0, 0), getSecondJointAngleRads(), m_enableThirdBodyMass) /
                                                calculateFJMomentOfInertia(m_enableThirdBodyMass);
        
                            xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                        }
                    return xdot;
                    },
                    currentXhat,
                    u,
                    dtSeconds);

            // We check for collision after updating xhat
            if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
                return VecBuilder.fill(fjMinAngleRads, 0);
            }
            if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
                return VecBuilder.fill(fjMaxAngleRads, 0);
            }
            return updatedXhat;
        }
    }

    public class SecondJointArmSim extends SingleJointedArmSim {
        DCMotor sjGearbox;
        double sjGearing;

        double sjMinAngleRads;
        double sjMaxAngleRads;

        public SecondJointArmSim(
            DCMotor gearbox,
            double gearing,
            double minAngleRads,
            double maxAngleRads,
            Matrix<N1, N1> measurementStdDevs) {
        super(gearbox,
                gearing,
                SingleJointedArmSim.estimateMOI(m_thirdBodyDistanceToCOGMeters, sjMassKg + m_thirdBodyMassKg),
                0, // Length of the arm is not needed since gravity calculation is overriden.
                minAngleRads,
                maxAngleRads,
                m_simulateGravity,
                measurementStdDevs);

            sjGearbox = gearbox;
            sjGearing = gearing;
            sjMinAngleRads = minAngleRads;
            sjMaxAngleRads = maxAngleRads;
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
            // The angle from horizontal is modifed by the angle of the first joint so
            // that gravity is applied correctly based on the multiple body system.
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
                        double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(((x.get(0, 0) - (Math.PI - getFirstJointAngleRads())))) / m_thirdBodyDistanceToCOGMeters;
                        xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                    }
                    return xdot;
                    },
                    currentXhat,
                    u,
                    dtSeconds);

            // We check for collision after updating xhat
            if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(sjMinAngleRads, 0);
            }
            if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(sjMaxAngleRads, 0);
            }
            return updatedXhat;
        }

    }


    private double calculateFJMomentOfInertia(boolean enableThirdBodyMass) {
        if (enableThirdBodyMass) {
            return fjArmMassKg * fjDistanceToCOGMeters * fjDistanceToCOGMeters + 
                    sjMassKg * sjDistanceToCOGMeters * sjDistanceToCOGMeters + 
                    m_thirdBodyMassKg * m_thirdBodyDistanceToCOGMeters * m_thirdBodyDistanceToCOGMeters;
        } else {
            return fjArmMassKg * fjDistanceToCOGMeters * fjDistanceToCOGMeters + 
                    sjMassKg * sjDistanceToCOGMeters * sjDistanceToCOGMeters;
        }
    }
}
