package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTagTest", group = OpModeGroups.TESTS)
public class AprilTagTestOpMode extends OpMode {
	MecanumDrive mecanumDrive;
	final static boolean USE_WEBCAM = true;
	public AprilTagProcessor aprilTag;
	private VisionPortal visionPortal; //for the driver hub camera view
	public int cameraMiddlePixel = 320;
	public Tag targetTag = Tag.PGP;

	/**
	 * A way to represent each tag and its id for this season
	 */
	public enum Tag {
		BLUE (20),
		GPP (21),
		PGP (22),
		PPG (23),
		RED (24);

		private final int id;

		// You actually cannot use a constructor for an Enum
		Tag(int id) {
			this.id = id;
		}
		public int id() {
			return this.id;
		}
	}

	@Override
	public void init() {
		hardwareMap.getClass(); // needed for setup somehow????
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		initAprilTag();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		//telemetryAprilTag();
		telemetry.addLine(String.valueOf(cameraMiddlePixel));
		telemetry.addData("tags found", aprilTag.getDetections().size());
		String ids = new String();
		for (AprilTagDetection tag : aprilTag.getDetections()) {
			ids += " " + tag.id;
		}
		telemetry.addData("tag ids", ids);
		for (AprilTagDetection tag:aprilTag.getDetections()) {
			if (tag.id == targetTag.id) {
				double tagOffset =  cameraMiddlePixel - tag.center.x;
				telemetry.addData("tag offset", tagOffset);
				telemetry.addLine("move " + ((tagOffset > 0) ? "left" : "right"));
			}
		}
		telemetry.update();
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
