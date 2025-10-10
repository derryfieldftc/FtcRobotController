package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@TeleOp(name = "Alternate drive test")
public class AlternateDriveTest extends OpMode {
	MecanumDrive mecanumDrive;

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));

	}

	@Override
	public void loop() {
		double drive = -gamepad1.left_stick_x;
		double strafe = -gamepad1.left_stick_y;
		double angular = -gamepad1.right_stick_x;


		PoseVelocity2d input = new PoseVelocity2d(new Vector2d(strafe, drive), angular);

		/*
		 * /----------\
		 * |B \    / R|
		 * ||  \  /  ||
		 * |    V     |
		 * |[]   ^  []|
		 * \----------/
		 *       X
		 * stand here looking up
		 */

		mecanumDrive.updatePoseEstimate();
		double rotation = 0; //mecanumDrive.localizer.getPose().heading.toDouble();
		double x_r = (input.linearVel.x * cos(rotation)) - (input.linearVel.x * sin(rotation));
		double y_r = (input.linearVel.y * cos(rotation)) + (input.linearVel.y * sin(rotation));

		telemetry.addData("x", input.linearVel.x);
		telemetry.addData("y", input.linearVel.y);
		telemetry.addData("dx", x_r);
		telemetry.addData("dy", y_r);
		telemetry.update();

		mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(x_r, y_r), input.angVel));
	}
}
