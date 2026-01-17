package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

@TeleOp
public class EncoderTest extends OpMode {
	DcMotor left, right, strafe;

	@Override
	public void init() {
		left = hardwareMap.dcMotor.get(RobotPart.Part.LeftDriveEncoder.name);
		right = hardwareMap.dcMotor.get(RobotPart.Part.RightDriveEncoder.name);
		strafe = hardwareMap.dcMotor.get(RobotPart.Part.StrafeEncoder.name);

		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		strafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	}

	@Override
	public void loop() {
		telemetry.addData("left", left.getCurrentPosition());
		telemetry.addData("right", right.getCurrentPosition());
		telemetry.addData("strafe", strafe.getCurrentPosition());
		telemetry.update();
	}
}
