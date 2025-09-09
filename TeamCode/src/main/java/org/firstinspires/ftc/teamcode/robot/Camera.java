package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A Camera to track april tags, please make sure to call init(), setTargetTag(), and loop()
 */
public class Camera {
	final static boolean USE_WEBCAM = true;
	private AprilTagProcessor aprilTag;
	private VisionPortal visionPortal; //for the driver hub camera view
	private int cameraMiddleX = 320; //default for the camera
	private int cameraMiddleY = 240; //default for the camera
	private double lastTagX = 0;
	private double lastTagY = 0;
	private double tagOffsetX = 0;
	private double tagOffsetY = 0;
	private Tag targetTag;
	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;

	/**
	 * Creates a new Camera, used to find april tags
	 * @param opMode
	 */
	public Camera(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	/**
	 * This method is a little funky, but it sets where you want to keep the tag within the camera,
	 * usually just the center of the camera
	 * @param XPixel
	 * @param YPixel
	 * @return
	 */
	public Camera setTagLocationTarget(int XPixel, int YPixel) {
		cameraMiddleX = XPixel;
		cameraMiddleY = YPixel;

		return this;
	}

	/**
	 * Sets the tag to search for and track
	 * @param targetTag
	 */
	public Camera setTargetTag(Tag targetTag) {
		this.targetTag = targetTag;
		return this;
	}

	/**
	 * Initializes the camera
	 */
	public void init() {
		initAprilTag();
	}

	/**
	 * Call this every time you wish to update the camera, normally once per loop
	 */
	public void loop() {
		//telemetryAprilTag();
		if (targetTag != null) {
			for (AprilTagDetection tag : aprilTag.getDetections()) {
				if (tag.id == targetTag.id) {
					tagOffsetX = cameraMiddleX - tag.center.x;
					tagOffsetY = cameraMiddleY - tag.center.y;
					lastTagX = tag.center.x;
					lastTagY = tag.center.y;
				}
			}
		}
	}

	public void telemetry() {
		telemetry.addData("tags found", aprilTag.getDetections().size());
		String ids = new String();
		for (AprilTagDetection tag : aprilTag.getDetections()) {
			ids += " " + tag.id;
		}
		telemetry.addData("tag ids", ids);
		if (targetTag != null) {
			telemetry.addData("X offset", tagOffsetX);
			telemetry.addLine("move " + ((tagOffsetX > 0) ? "left" : "right"));
			telemetry.addData("Y offset", tagOffsetY);
			telemetry.addLine("move " + ((tagOffsetY > 0) ? "up" : "down"));
		}
		telemetry.update();

	}

	/**
	 * Get the offset of the tag from your target position, negative means left
	 * @return
	 */
	public double getOffsetX() {
		return tagOffsetX;
	}

	/**
	 * Get the offset of the tag from your target position, negative means down
	 * @return
	 */
	public double getOffsetY() {
		return tagOffsetY;
	}

	/**
	 * Get what x pixel the tag was last recognized
	 * @return
	 */
	public double getLastTagX() {
		return lastTagX;
	}

	/**
	 * Get what y pixel the tag was last recognized
	 * @return
	 */
	public double getLastTagY() {
		return lastTagY;
	}

	// thank you to the concept april tag file
	private void initAprilTag() {
		aprilTag = new AprilTagProcessor.Builder()
				.setDrawAxes(false)
				.setDrawCubeProjection(false)
				.setDrawTagOutline(true)
				.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.build();

		// Adjust Image Decimation to trade-off detection-range for detection-rate.
		// eg: Some typical detection data using a Logitech C920 WebCam
		// Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
		// Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
		// Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
		// Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
		// Note: Decimation can be changed on-the-fly to adapt during a match.
		//aprilTag.setDecimation(3);

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		if (USE_WEBCAM) {
			// NOTE FOR WEBCAM, MAKE SURE IT IS PLUGGED INTO LEFTMOST USB, OTHERWISE THE CONNECTION IS IFFY
			builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
		} else {
			builder.setCamera(BuiltinCameraDirection.BACK);
		}

		// Set and enable the processor.
		builder.addProcessor(aprilTag);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);
	}


	/**
	 * Add telemetry about AprilTag detections.
	 */
	private void telemetryAprilTag() {

		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		// Step through the list of detections and display info for each one.
		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
				telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
				telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
			} else {
				telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
				telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
			}
		}

		// Add "key" information to telemetry
		telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
		telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
		telemetry.addLine("RBE = Range, Bearing & Elevation");

	}
}
