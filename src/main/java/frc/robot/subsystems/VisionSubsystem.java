// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.AdvantageUtil;

public class VisionSubsystem extends SubsystemBase {
	private final int LARGEST_APRILTAG_ID = 8;
	private final int NUM_SAMPLES_TO_AVERAGE = 13;
	private final double DISTANCE_AT_CONVERSION = 1.43;
	private final int PIXELS_AT_CONVERSION = 52;
	private final int NUM_SAMPLES_TO_WAIT = 10;

	/** The amount of time that passes with no data before the filter should be reset. */
	private final double DURATION_TO_RESET = 5;

	private final CameraData[] m_photonCameras = {
		new CameraData("OV9281", new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(0))),
	};

	private static VisionSubsystem INSTANCE;
	public static VisionSubsystem getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new VisionSubsystem();
		}
		return INSTANCE;
	}

	private TranslationAverage[] m_apriltagFilters = new TranslationAverage[LARGEST_APRILTAG_ID];
	private Pose2d[] m_targetPoses = new Pose2d[LARGEST_APRILTAG_ID];
	private DoubleArrayPublisher[] pubTranslations = new DoubleArrayPublisher[LARGEST_APRILTAG_ID];
	private DoubleArrayPublisher[] pubAverageTranslations = new DoubleArrayPublisher[LARGEST_APRILTAG_ID];
	private DoublePublisher[] pubDistances = new DoublePublisher[LARGEST_APRILTAG_ID];
	private DoublePublisher[] pubYaws = new DoublePublisher[LARGEST_APRILTAG_ID];
    private AprilTagFieldLayout m_apriltagLayout;
    private boolean m_failedToLoadFieldLayout = false;
	
	/** Creates a new VisionSubsystem. */
	private VisionSubsystem() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("VisionSubsystem");
		for (int i = 0; i < m_apriltagFilters.length; i++) {
			m_apriltagFilters[i] = new TranslationAverage(NUM_SAMPLES_TO_AVERAGE);

			String subtable = "Tag" + i + "/";
			pubTranslations[i] = table.getDoubleArrayTopic(subtable+"TranslationTag").publish(PubSubOption.periodic(0.02));
			pubAverageTranslations[i] = table.getDoubleArrayTopic(subtable+"AverageTranslationTag").publish(PubSubOption.periodic(0.02));
			pubDistances[i] = table.getDoubleTopic(subtable+"Distance").publish(PubSubOption.periodic(0.02));
			pubYaws[i] = table.getDoubleTopic(subtable+"Yaw").publish(PubSubOption.periodic(0.02));
		}

		for (CameraData camera : m_photonCameras) {
			camera.createPhotonCamera();
		}

		try {
            m_apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            m_failedToLoadFieldLayout = true;
        } 

		for (int i = 0; i < m_targetPoses.length; i++) {
			Optional<Pose3d> optionalPose = m_apriltagLayout.getTagPose(i);
			if (optionalPose.isEmpty()) {
				m_targetPoses[i] = new Pose2d();
				continue;
			}
			m_targetPoses[i] = optionalPose.get().toPose2d();
		}
	}

	@Override
	public void periodic() {
		for (CameraData camera : m_photonCameras) {
			PhotonPipelineResult result = camera.getPhotonCamera().getLatestResult();
			for (PhotonTrackedTarget target : result.targets) {
				if (target.getFiducialId() < 0 || target.getFiducialId() > LARGEST_APRILTAG_ID) {
					continue;
				}

				double distance = calcDistance(target.getDetectedCorners());
				pubDistances[target.getFiducialId()].accept(distance);

				pubYaws[target.getFiducialId()].accept(target.getYaw());
				
				Translation2d newMeasurement = calcRobotToTarget(
					distance,
					Rotation2d.fromDegrees(target.getYaw()),
					SwerveSubsystem.getInstance().getHeading(),
					camera.getPose()
				);

				Optional<Pose3d> optionalPose = m_apriltagLayout.getTagPose(target.getFiducialId());
				if (optionalPose.isEmpty()) {
					continue;
				}

				Pose2d tagPose = optionalPose.get().toPose2d();
				Rotation2d targetHeading = tagPose.getRotation();

				pubTranslations[target.getFiducialId()].accept(
					AdvantageUtil.deconstruct(
						new Pose2d(newMeasurement, targetHeading)));

				Translation2d filteredTranslation = 
					m_apriltagFilters[target.getFiducialId()]
						.calculate(newMeasurement);

				pubAverageTranslations[target.getFiducialId()].accept(
					AdvantageUtil.deconstruct(
						new Pose2d(filteredTranslation, targetHeading)));

				m_targetPoses[target.getFiducialId()] = new Pose2d(
					filteredTranslation, 
					targetHeading);
			}
		}

		double currentTime = Timer.getFPGATimestamp();
		for (TranslationAverage filter : m_apriltagFilters) {
			if (currentTime - filter.getTimeSinceLastData() > DURATION_TO_RESET) {
				filter.resetFilters();
			}
		}
	}

	public Pose2d getTargetPose(int tagId) {
		return m_targetPoses[tagId];
	}

	public int getNumSamples(int tagId) {
		return m_apriltagFilters[tagId].getNumSamples();
	}

	public double getTimeSinceLastData(int tagId) {
		return m_apriltagFilters[tagId].getTimeSinceLastData();
	}

	public void resetFilters() {
		for (TranslationAverage filters : m_apriltagFilters) {
			filters.resetFilters();
		}
	}

	public double calcDistance(List<TargetCorner> corners) {
		double avgPixelHeight = ((corners.get(0).y - corners.get(3).y) +
				corners.get(1).y - corners.get(2).y) / 2.0;

		return (DISTANCE_AT_CONVERSION / (avgPixelHeight / PIXELS_AT_CONVERSION));
	}

	public Rotation2d calcHeadingToFaceApriltag(int tagId) {
		Pose2d targetPose = getTargetPose(tagId);
		Pose2d camera = SwerveSubsystem.getInstance().getPose().plus(
			new Transform2d(
				new Pose2d(), 
				m_photonCameras[tagId].getPose()));
		
		Translation2d translation = targetPose.getTranslation().minus(camera.getTranslation());

		return new Rotation2d(
			Math.atan2(translation.getY(), translation.getX()));
	}

	public CommandBase getWaitForVisionCommand(int tagId) {
		return new FunctionalCommand(
			// () -> this.resetFilters(), 
			() -> {},
			() -> {}, 
			(interrupted) -> {}, 
			() -> this.getNumSamples(tagId) >= NUM_SAMPLES_TO_WAIT,
			this
		);
	}

	/**
	 * Calculate the a translation2d in the field coordinate frame from the robot to
	 * the target.
	 *
	 * @param visionDistance Distance from the camera to the target in the camera's
	 *                       coordinate frame
	 * @param visionYaw      Yaw of the camera to the target in the camera's
	 *                       coordinate frame
	 * @param heading        Heading of the robot measured by the gyro
	 * @param fieldToTarget  Pose3d of the target in the field's coordinate frame.
	 *                       Use AprilTagFieldLayout.java for this.
	 * @param robotToCamera  Pose2d offset of the camera in the robot's coordinate
	 *                       frame
	 *
	 * @return Translation in the field's coordinate frame between the robot and the
	 *         target.
	 */
	private static Translation2d calcRobotToTarget(double visionDistance, Rotation2d visionYaw, Rotation2d heading,
			Pose2d robotToCamera) {
		// Convert distance and yaw to XY
		Translation2d visionXY = new Translation2d(visionDistance, visionYaw);

		// Rotate XY from the camera's coordinate frame to the robot's coordinate frame
		Translation2d cameraToTargetRELATIVE = visionXY.rotateBy(robotToCamera.getRotation());

		// Add the XY from the robot center to camera and from the camera to the target
		Translation2d robotToTargetRELATIVE = robotToCamera.getTranslation().plus(cameraToTargetRELATIVE);

		// Rotate XY from the robot's coordinate frame to the field's coordinate frame
		Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(heading);

		return robotToTarget;
	}

	public class TranslationAverage {
		private final int DEFAULT_SAMPLES = 10;
		private final LinearFilter xFilter;
		private final LinearFilter yFilter;
		private Translation2d currentTranslation;
		private int numSamples = 0;
		private double timeSinceLastData;
		private boolean isReset;

		public TranslationAverage(int size) {
			if (size < 1) {
				size = 10;
				DriverStation.reportError("TranslationAverage was told to do a rolling average " +
						"of less than 1 sample. Defaulting to " + DEFAULT_SAMPLES + " samples.", true);
			}
			xFilter = LinearFilter.movingAverage(size);
			yFilter = LinearFilter.movingAverage(size);
		}

		public Translation2d calculate(Translation2d newMeasurement) {
			currentTranslation = new Translation2d(
					xFilter.calculate(newMeasurement.getX()),
					yFilter.calculate(newMeasurement.getY()));

			numSamples++;
			timeSinceLastData = Timer.getFPGATimestamp();
			isReset = false;
			return currentTranslation;
		}

		public void resetFilters() {
			if (!isReset) {
				xFilter.reset();
				yFilter.reset();
				isReset = true;
			} 
		}

		public int getNumSamples() {
			return numSamples;
		}

		public double getTimeSinceLastData() {
			return timeSinceLastData;
		}
	}

	public class CameraData {
		private Pose2d cameraLocation;
		private String photonCameraName;
		private PhotonCamera photonCamera;

		public CameraData(String name, Pose2d location) {
			cameraLocation = location;
			photonCameraName = name;
		}

		public Pose2d getPose() {
			return cameraLocation;
		}

		public String getName() {
			return photonCameraName;
		}

		public void createPhotonCamera() {
			photonCamera = new PhotonCamera(photonCameraName);
		}

		public PhotonCamera getPhotonCamera() {
			return photonCamera;
		}
	}
}
