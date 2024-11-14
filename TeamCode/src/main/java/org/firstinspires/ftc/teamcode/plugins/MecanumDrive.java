package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.util.function.Function;
@ExportClassToBlocks
public class MecanumDrive extends RobotPlugin {
	OpMode opMode;
	Telemetry telemetry;
	HardwareMap hardwareMap;
	Gamepad gamepad1;
	Gamepad gamepad2;
	DcMotor motorFL;
	DcMotor motorBL;
	DcMotor motorFR;
	DcMotor motorBR;

	/**
	 * This class is a simple mecanum drive. The motors run without encoders so it should "just work"
	 */
	public MecanumDrive(OpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad1 = opMode.gamepad1;
		this.gamepad2 = opMode.gamepad2;
	}

	@Override
	public void init() {
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorFR = hardwareMap.dcMotor.get("motorFR");
		motorBR = hardwareMap.dcMotor.get("motorBR");

		motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
		motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
	}

	public void loop() {
		double y = -gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = gamepad1.right_stick_x;
		double scale_factor = 0.50;
		if (gamepad2.left_bumper) {
			y = -gamepad2.left_stick_y;
			x = gamepad2.left_stick_x;
			rx = gamepad2.right_stick_x;

			// scale output
			x = scale_factor * x;
			y = scale_factor * y;
			rx = scale_factor * rx;
		} else {
			y = -gamepad1.left_stick_y;
			x = gamepad1.left_stick_x;
			rx = gamepad1.right_stick_x;
		}

		double powerFL = y + x + rx;
		double powerBL = y - x + rx;
		double powerFR = y - x - rx;
		double powerBR = y + x - rx;

		powerFL = clamp(1, -1, powerFL);
		powerBL = clamp(1, -1, powerBL);
		powerFR = clamp(1, -1, powerFR);
		powerBR = clamp(1, -1, powerBR);

		motorFL.setPower(powerFL);
		motorBL.setPower(powerBL);
		motorFR.setPower(powerFR);
		motorBR.setPower(powerBR);

	}

	private double clamp(double max, double min, double num) {
		if (num > max) {
			return max;
		} else if (num < min) {
			return min;
		} else {
			return num;
		}
	}
}
