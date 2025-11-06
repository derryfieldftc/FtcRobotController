package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
public class Lift extends RobotPart {
	Servo lift;
	enum LiftPosition {
		Up (.1),
		Down (.5);
		public double pos;
		LiftPosition(double pos) {this.pos = pos;};
	}

	public Lift(OpMode opMode) {
		super(opMode);
	}

	public Lift setPosition(LiftPosition pos) {
		lift.setPosition(pos.pos);
		return this;
	}
}
