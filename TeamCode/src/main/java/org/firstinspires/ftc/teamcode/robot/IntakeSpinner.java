package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSpinner {
	HardwareMap hardwareMap;
	OpMode opMode;
	Servo spinnerSpinner;
	CRServo spinnerR, spinnerL;

	public IntakeSpinner(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
	}

	public void init() {
		spinnerSpinner = hardwareMap.servo.get("spinnerspinner");
		spinnerL = hardwareMap.crservo.get("spinnerL");
		spinnerR = hardwareMap.crservo.get("spinnerR");
		spinnerR.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void setPower(double power) {
		spinnerR.setPower(power);
		spinnerL.setPower(power);
	}

	public void setRotation(double rotation) {
		spinnerSpinner.setPosition(rotation);
	}
}
