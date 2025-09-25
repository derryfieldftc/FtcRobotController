package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

import java.util.ArrayList;

@TeleOp(name = "RR_DrawPath", group = OpModeGroups.TESTS)
public class RRDrawPath extends OpMode {
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	ArrayList<Pose2d> poses = new ArrayList<>();


	@Override
	public void init() {
		//Middle of the field aiming straight up
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
		mgamepad = new GamepadManager(gamepad1);
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		double drive = -gamepad1.left_stick_x;
		double strafe = -gamepad1.left_stick_y;
		double angular = -gamepad1.right_stick_x;

		telemetry.addLine(String.format("Powers: x: %.3f, y: %f, a: %.3f", strafe, drive, angular));

		mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(strafe, drive), angular));
		mecanumDrive.updatePoseEstimate();

		Pose2d pose = mecanumDrive.localizer.getPose();
		telemetry.addLine(String.format("x: %.3f, y: %.3f, r: %.3f", pose.position.x, pose.position.y, pose.heading.toDouble()));
		telemetry.update();

		mgamepad.poll();

		if (mgamepad.justPressed(GamepadManager.Button.Y)) {
			poses = new ArrayList<>();
			mecanumDrive.localizer.setPose(new Pose2d(0, 0, Math.PI / 2));
		}

		if (mgamepad.justPressed(GamepadManager.Button.A)) {
			Pose2d pose_copy = new Pose2d(mecanumDrive.localizer.getPose().position.x, mecanumDrive.localizer.getPose().position.y, mecanumDrive.localizer.getPose().heading.toDouble());
			poses.add(pose_copy);
		}

		if (mgamepad.justPressed(GamepadManager.Button.B)) {
			if (!poses.isEmpty())
				poses.remove(poses.size() - 1);
		}

		for (int i = poses.size() - 1; i >= 0; i--) {
			Pose2d temp = poses.get(i);
			telemetry.addLine(String.format("(%d): x: %.3f, y: %.3f", i, temp.position.x, temp.position.y, temp.heading.toDouble()));
		}
	}
}
