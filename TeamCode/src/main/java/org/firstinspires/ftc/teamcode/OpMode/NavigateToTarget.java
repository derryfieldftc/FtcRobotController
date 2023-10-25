package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This example op mode will navigate to a specific AprilTag.
 * It uses a proportional control to try to maintain target
 * values for the x, y, and yaw pose values for a specific tag.
 */
@TeleOp(name="NavigateToTarget", group="Tests")
public class NavigateToTarget extends LinearOpMode {

    final double DESIRED_Y = 19;      // inches.
    final double DESIRED_X = 0.0;       // inches.
    final double DESIRED_YAW = 0.0;     // degrees.
    final double TRANSLATION_THRESHOLD = 1.0;   // inches
    final double YAW_THRESHOLD = 2.0;           // degrees

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.2  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.1 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.025  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.15;   //  Clip the turn speed to this max value (adjust for your robot)

    public static final String rightFrontName = "motorFR";
    public static final String leftFrontName = "motorFL";
    public static final String rightRearame = "motorBR";
    public static final String leftRearName = "motorBL";

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int ALTERNATE_TAG_ID = 9;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    MecanumDrive bot;

    @Override
    public void runOpMode() {
        boolean targetFound         = false;        // Set to true when an AprilTag target is detected
        double  forward             = 0;            // Desired forward power/speed (-1 to +1)
        double  strafe              = 0;            // Desired strafe power/speed (-1 to +1)
        double  rotate              = 0;            // Desired turning power/speed (-1 to +1)

        // get a reference to our mecanum drive.
        bot = new MecanumDrive(hardwareMap, rightFrontName, leftFrontName, rightRearame, leftRearName);

        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // wait for user to press start button on Driver Hub.
        waitForStart();

        while (opModeIsActive()) {
            // we haven't found the target yet.
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID) || detection.id == ALTERNATE_TAG_ID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("y",  "%5.1f inches", desiredTag.ftcPose.y);
                telemetry.addData("x",  "%5.1f inches", desiredTag.ftcPose.x);
                telemetry.addData("yaw",  "%5.1f inches", desiredTag.ftcPose.yaw);

            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  yError      = (desiredTag.ftcPose.y - DESIRED_Y);
                double  xError      = (desiredTag.ftcPose.x - DESIRED_X);
                double  yawError        = desiredTag.ftcPose.yaw - DESIRED_YAW;

                // I added these conditional statements to try and quell some of the limit cycling
                // but I don't think it actually helps.  The April Tags
                // seem to give spurious (and large) error values occasionally.
                if (Math.abs(yError) < TRANSLATION_THRESHOLD) {
                    yError = 0.0;
                }

                if (Math.abs(xError) < TRANSLATION_THRESHOLD) {
                    xError = 0.0;
                }

                if (Math.abs(yawError) < YAW_THRESHOLD) {
                    yawError = 0.0;
                }

                telemetry.addData("yError",  "%5.1f inches", yError);
                telemetry.addData("xError",  "%5.1f inches", xError);
                telemetry.addData("yawError",  "%5.1f degrees", yawError);


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                forward  = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                rotate   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe  = Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                bot.drive(forward, strafe, rotate);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forward, strafe, rotate);
            } else {
                // use gamepad input to drive.
                forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
                strafe = gamepad1.left_stick_x;   // Perfect child, no flip
                rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)

                bot.drive(forward, strafe, rotate);

                telemetry.addData("forward", forward);
                telemetry.addData("strafe", forward);
                telemetry.addData("rotate", forward);
            }

            telemetry.update();
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
