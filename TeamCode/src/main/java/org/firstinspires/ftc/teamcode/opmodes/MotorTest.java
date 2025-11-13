package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends OpMode {
	DcMotor br, bl, fr, fl;
	@Override
	public void init() {
		br = hardwareMap.dcMotor.get("motorBR");
		fl = hardwareMap.dcMotor.get("motorFL");
		fr = hardwareMap.dcMotor.get("motorFR");
		bl = hardwareMap.dcMotor.get("motorBL");

	}

	@Override
	public void loop() {
		fr.setPower(gamepad1.right_trigger);
		fl.setPower(gamepad1.left_trigger);

		br.setPower(gamepad2.right_trigger);
		bl.setPower(gamepad2.left_trigger);

	}
}
