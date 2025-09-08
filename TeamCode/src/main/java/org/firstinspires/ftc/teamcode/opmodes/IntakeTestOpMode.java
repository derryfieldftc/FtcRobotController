package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;

@TeleOp(name="IntakeTestOpMode", group = OpModeGroups.TESTS)
public class IntakeTestOpMode extends OpMode {
	RobotPlugin mechanumDrive;
	DcMotor intake;
	boolean intakeOn = false;
	double intakePower = 0;

	@Override
	public void init() {
		hardwareMap.getClass(); //for setup
		mechanumDrive = new MecanumDrive(this);
		mechanumDrive.init();
		intake = hardwareMap.dcMotor.get("intake");
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void loop() {
		mechanumDrive.loop();
		if (gamepad1.a)
			intakeOn = true;
		if (gamepad1.b)
			intakeOn = false;

		if (intakeOn) {
			intake.setPower(gamepad1.right_stick_x);
		}

		telemetry.addData("intakePower", intakePower);
		telemetry.addData("intakeOn", intakeOn);
	}
}
