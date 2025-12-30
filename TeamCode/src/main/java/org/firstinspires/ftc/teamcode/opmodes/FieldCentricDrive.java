package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class FieldCentricDrive extends OpMode {
	Follower follower;
	@Override
	public void init() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(0, 0, 0));
	}

	@Override
	public void loop() {
		follower.update();
		follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
	}
}
