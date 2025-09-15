package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OpModeGroups;

@TeleOp(name = "AlexExampleMecanum", group = OpModeGroups.TESTS)
public class AlexFirstOpMode extends OpMode {
	DcMotor motorFR, motorFL, motorBR, motorBL;
	Gamepad gamepad;

	@Override
	public void init() {
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorBR = hardwareMap.dcMotor.get("motorBR");
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorFR = hardwareMap.dcMotor.get("motorFR");

		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

		gamepad = gamepad1;
	}

	@Override
	public void loop() {
		double y = -gamepad.left_stick_y;
		double x = gamepad.left_stick_x;
		double rx = gamepad.right_stick_x;

		double powerFL = y + x + rx;
		double powerBL = y - x + rx;
		double powerFR = y - x - rx;
		double powerBR = y + x - rx;

		motorFL.setPower(powerFL);
		motorBL.setPower(powerBL);
		motorFR.setPower(powerFR);
		motorBR.setPower(powerBR);

		telemetry.addData("x", x);
		telemetry.addData("y", y);
		telemetry.addData("rx", rx);
		telemetry.update();
	}
}
