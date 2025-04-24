package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {
	DcMotor drive;
	DcMotor strafe;

	@Override
	public void runOpMode() throws InterruptedException {
		drive = hardwareMap.dcMotor.get("drive");
		strafe = hardwareMap.dcMotor.get("strafe");
		drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		strafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		waitForStart();
		while (opModeIsActive()) {
			telemetry.addData("drive", drive.getCurrentPosition());
			telemetry.addData("strafe", strafe.getCurrentPosition());
			telemetry.update();
		}
	}
}
