package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;

@TeleOp(name = "SpinnerServoTest", group = OpModeGroups.TESTS)
public class SpinnerTest extends OpMode {
	MecanumDrive drive;
	GamepadManager mgamepad2;
	Servo spinnerSpinner;
	CRServo spinnerL, spinnerR;

	@Override
	public void init() {
		drive = new MecanumDrive(this);
		drive.init();
		mgamepad2 = new GamepadManager(gamepad2);
		spinnerL = hardwareMap.get(CRServo.class, "spinnerL");
		spinnerR = hardwareMap.get(CRServo.class, "spinnerR");
		spinnerSpinner = hardwareMap.servo.get("spinnerspinner");

	}

	@Override
	public void loop() {
		mgamepad2.poll();
		drive.loop();

		spinnerL.setDirection(DcMotorSimple.Direction.REVERSE);
		spinnerR.setDirection(DcMotorSimple.Direction.FORWARD);
		spinnerR.setPower(gamepad2.right_stick_y);
		spinnerL.setPower(gamepad2.left_stick_y);
		spinnerSpinner.setPosition(gamepad1.right_stick_y);
	}
}
