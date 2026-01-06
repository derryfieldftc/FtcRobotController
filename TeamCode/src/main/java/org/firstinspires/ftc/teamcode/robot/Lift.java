package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift extends RobotPart {
	Servo lift;

	public Lift(OpMode opMode) {
		super(opMode);
	}

	public void init() {
		lift = hardwareMap.servo.get(Part.LiftServo.name);
	}

	public enum Position {
		Up		(.4),
		Down	(0);

		final double position;
		Position(double position) {
			this.position = position;
		}
	}

	public void setPosition(Position position) {
		lift.setPosition(position.position);
	}
}
