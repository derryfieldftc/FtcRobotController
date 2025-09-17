package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Deprecated
public class IntakeSpinner {
	HardwareMap hardwareMap;
	OpMode opMode;
	public Servo spinnerSpinner;
	CRServo spinnerR, spinnerL;
	double rotationLimit = .35;
	double theoRotator = .5;

	ColorSensor colorSensor;

	public IntakeSpinner(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
	}

	public void init() {
		colorSensor = hardwareMap.colorSensor.get("spinnercolorsensor");
		colorSensor.enableLed(true);
		spinnerSpinner = hardwareMap.servo.get("spinnerspinner");
		spinnerL = hardwareMap.crservo.get("spinnerL");
		spinnerR = hardwareMap.crservo.get("spinnerR");
		spinnerL.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void setPower(double power) {
		spinnerR.setPower(power);
		spinnerL.setPower(power);
	}

	public void setRotation(double rotation) {
		spinnerSpinner.setPosition(Math.max(rotation, rotationLimit));
	}

	public void theoRotate(double delta) {
		theoRotator = clamp(theoRotator + delta / 10, 0, 1);
		spinnerSpinner.setPosition(theoRotator);
	}

	//TODO! this is bad. do not this, do something that is not this and test more
	public Field.Ball getBallColor() {
		if (colorSensor.green() > 1000) {
			return Field.Ball.Green;
		} else if (colorSensor.blue() > 500) {
			return Field.Ball.Purple;
		}
		return Field.Ball.None;
	}

	public void dumpColor() {
		Telemetry telemetry = opMode.telemetry;
		telemetry.addLine(String.format("Color, R: %d G: %d B: %d ", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
	}
}
