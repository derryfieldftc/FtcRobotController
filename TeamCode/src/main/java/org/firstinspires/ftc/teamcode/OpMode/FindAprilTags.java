package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "FindAprilTags", group = "Sample")
public class FindAprilTags extends LinearOpMode {

    VisionPortal camera;
    AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		while (opModeIsActive()) {
			telemetry.addData("Status", "Running");
			telemetry.update();
        }
    }
}
