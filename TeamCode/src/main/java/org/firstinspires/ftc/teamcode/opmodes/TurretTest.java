package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose2d;

@TeleOp(name = "TurretTest", group = OpModeGroups.TESTS)
public class TurretTest extends OpMode {
	org.firstinspires.ftc.teamcode.RR.MecanumDrive mecanumDrive;
	Turret turret;

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
		turret = new Turret(this, new TurretPose2d(new Pose2d(0, 0, Math.PI / 2), 0)).useGamepad()
				.trackTarget()
				.updatePose(new Pose2d(0, 0, Math.PI / 2))
				.setTarget(new Vector2d(0, 0));

		turret.init();
	}

	@Override
	public void loop() {
		double drive = -gamepad1.left_stick_x;
		double strafe = -gamepad1.left_stick_y;
		double angular = -gamepad1.right_stick_x;

		if (gamepad1.y) {
			mecanumDrive.localizer.setPose(new Pose2d(0, 0, 0));
		}

		if (gamepad1.a) {
			turret.setTarget(mecanumDrive.localizer.getPose().position);
		}

		telemetry.addLine(String.format("Powers: x: %.3f, y: %f, a: %.3f", strafe, drive, angular));

		mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(strafe, drive), angular));
		mecanumDrive.updatePoseEstimate();
		turret.updatePose(mecanumDrive.localizer.getPose());
		turret.loop();
	}
}
