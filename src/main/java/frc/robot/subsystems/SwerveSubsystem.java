package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem INSTANCE;

  private final PigeonIMU m_pigeon;

  private SwerveDriveOdometry m_odometry;
  private SwerveModule[] m_swerveModules;

  private ProfiledPIDController m_xPid;
  private ProfiledPIDController m_yPid;
  private ProfiledPIDController m_rotPid;

  /**
   * Get the singleton instance of the SwerveSubsystem.
   * 
   * @return Instance of SwerveSubsystem
   */
  public static SwerveSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SwerveSubsystem();
    }
    return INSTANCE;
  }

  /* Construct the Swerve Subsystem */
  private SwerveSubsystem() {
    m_pigeon = new PigeonIMU(Config.CANID.PIGEON);
    m_pigeon.configFactoryDefault();

    m_swerveModules = new SwerveModule[] {
        new SwerveModule(0, Config.Swerve.Mod0.CONSTANTS),
        new SwerveModule(1, Config.Swerve.Mod1.CONSTANTS),
        new SwerveModule(2, Config.Swerve.Mod2.CONSTANTS),
        new SwerveModule(3, Config.Swerve.Mod3.CONSTANTS)
    };

    m_odometry = new SwerveDriveOdometry(
        Config.Swerve.SWERVE_KINEMATICS,
        getYaw(),
        getPositions(),
        new Pose2d());

    m_xPid = createPidFromConstants(Config.Swerve.TRANSLATION_PID, Config.Swerve.TRANSLATION_PID_SPEEDS);
    m_yPid = createPidFromConstants(Config.Swerve.TRANSLATION_PID, Config.Swerve.TRANSLATION_PID_SPEEDS);
    m_rotPid = createPidFromConstants(Config.Swerve.ROTATION_PID, Config.Swerve.ROTATION_PID_SPEEDS);
    m_rotPid.enableContinuousInput(0, 2 * Math.PI);
  }

  /**
   * Stop the swerve motors until a new instruction comes.
   */
  public void stopMotors() {
    for (SwerveModule mod : m_swerveModules) {
      mod.stopMotors();
    }
  }

  /**
   * Drive swerve with given speeds.
   * 
   * Follow WPILib standards for +X, +Y, +rot directions:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
   * 
   * @param xSpeed        Speed in the X direction in meters per second.
   * @param ySpeed        Speed in the Y direction in meters per second.
   * @param rotSpeed      Angular Speed in radians per second.
   * @param fieldRelative Whether to be field relative or robot relative.
   * @param isOpenLoop    Whether to be open loop or closed loop contorl.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) {
      swerveModuleStates = Config.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              rotSpeed,
              getHeading()));
    } else {
      swerveModuleStates = Config.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xSpeed,
              ySpeed,
              rotSpeed));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.Swerve.MAX_ATTAINABLE_SPEED);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the states of the modules directly.
   * 
   * @param desiredStates Array of 4 SwerveModuleStates (vel and steering angle)
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.Swerve.MAX_ATTAINABLE_SPEED);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Get the pose of the robot from odometry.
   * 
   * @return Pose2d of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Reset odometry to the given pose.
   * 
   * @param pose Pose to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getYaw(),
        getPositions(),
        pose);
  }

  /**
   * Get the states (drive vel & steering angle) of all modules.
   * 
   * @return Array of SwerveModuleStates.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Get the states (drive position & steering angle) of all modules.
   * 
   * @return Array of SwerveModulePosition.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Returns the heading of the robot.
   * 
   * This is private because only the odoemtry get's the raw gyro value.
   * Everything else get's the gyro value from odometry since it does an
   * offset.
   *
   * @return the robot's heading as a Rotation2d, from -180 to 180
   */
  private Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  /**
   * Returns the heading of the robot.
   * 
   * Uses this method for heading. Odometry does an offset to ensure this has the
   * correct origin.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public ChassisSpeeds getSpeeds(boolean fieldRelative) {
    ChassisSpeeds speeds = Config.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getStates());

    if (!fieldRelative) {
      return speeds;
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading().unaryMinus());
    }
  }
  
  public void resetDriveToPose() {
    ChassisSpeeds speeds = getSpeeds(true);

    m_xPid.reset(getPose().getX(), speeds.vxMetersPerSecond);
    m_yPid.reset(getPose().getY(), speeds.vyMetersPerSecond);
    m_rotPid.reset(getHeading().getRadians(), speeds.omegaRadiansPerSecond);
  }

  public void driveToPose(Pose2d pose) {
    double xSpeed = m_xPid.calculate(getPose().getX(), pose.getX());
    double ySpeed = m_yPid.calculate(getPose().getY(), pose.getY());
    double rotSpeed = m_rotPid.calculate(getHeading().getRadians(), pose.getRotation().getRadians());

    xSpeed += m_xPid.getSetpoint().velocity;
    ySpeed += m_yPid.getSetpoint().velocity;
    rotSpeed += m_rotPid.getSetpoint().velocity;

    drive(xSpeed, ySpeed, rotSpeed, true, false);
  }

  @Override
  public void periodic() {
    m_odometry.update(getYaw(), getPositions());
  }

  /**
   * Returns a command that resets the heading to the given heading when
   * scheduled.
   * 
   * Maintains the current X and Y value of odometry.
   * 
   * @param newHeading Desired heading to reset to.
   * @return A command to reset odometry
   */
  public CommandBase getResetHeadingCommand(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> resetOdometry(
            new Pose2d(
                getPose().getTranslation(),
                newHeading)));
  }

  private ProfiledPIDController createPidFromConstants(PIDConstants constants, Constraints constraints) {
    return new ProfiledPIDController(
      constants.kP, 
      constants.kI, 
      constants.kD, 
      constraints);
  }
}