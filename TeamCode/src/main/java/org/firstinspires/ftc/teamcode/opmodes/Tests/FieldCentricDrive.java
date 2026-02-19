package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
@Disabled
public class FieldCentricDrive extends OpMode {
	Follower follower;
	@Override
	public void init() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(0, 0, 0));
	}

	@Override
	public void start() {
		follower.startTeleOpDrive();
	}

	@Override
	public void loop() {
		follower.update();
		follower.setTeleOpDrive( // values for blue
				gamepad1.left_stick_y * (1 - gamepad1.right_trigger) * ((gamepad1.right_bumper) ? -1 : 1),
				gamepad1.left_stick_x * (1 - gamepad1.right_trigger) * ((gamepad1.right_bumper) ? -1 : 1),
				-gamepad1.right_stick_x * (1 - gamepad1.right_trigger),
				gamepad1.right_bumper);
	}
}
