package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindexer extends RobotPart {
	DcMotorEx rotator;
	Servo lift;
	Position currentPosition = Position.Zero;

	// 4:1 ratio
	// 103.8 ticks per revolution
	// so to spin large gear its 415.2 ticks
	final int fullRotationTicks = 415;
	final int stepTicks = 138;
	// follows right hand rule, so 0, 1, 2

	public enum Position {
		Zero	(0),
		One		(138),
		Two		(277);

		public int fromZeroTicks;
		Position(int ticks) {
			this.fromZeroTicks = ticks;
		}
	}

	public enum Height {
		Down	(0),
		Up		(1);

		public double height;
		Height(double height) {
			this.height = height;
		}
	}

	public Spindexer(OpMode opMode) {
		super(opMode);
	}

	public void init() {
		rotator = hardwareMap.get(DcMotorEx.class, Part.SpindexerRotator.name);
		rotator.setTargetPosition(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		// What the heck is happening with the PIDF. We must use 0 for p i and d if we use a non-depricated algorithm
//		rotator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(.5, 1, 0.001, .02, MotorControlAlgorithm.LegacyPID));
		lift = hardwareMap.servo.get(RobotPart.Part.SpindexerLift.name);
	}

	public void setRotatorPower(double power) {
		rotator.setPower(power);
	}

	public void setLiftPosition(Height height) {
		lift.setPosition(height.height);
	}

	public void moveToReset() {
		//TODO! make this reset the encoder position and zero the spindexer
	}

	/**
	 * Spins to whichever position
	 * @param targetPosition
	 */
	public void setPosition(Position targetPosition) {
		int currentPositionTicks = rotator.getCurrentPosition();
		int rotations = currentPositionTicks / fullRotationTicks;

		// Don't look at this
		switch (currentPosition) {
			case Zero:
				int fakeZero = rotations * fullRotationTicks;
				switch (targetPosition) {
					case Zero:
						break;
					case One:
						rotator.setTargetPosition(fakeZero + stepTicks);
						break;
					case Two:
						rotator.setTargetPosition(fakeZero - stepTicks);
						break;
				}
				break;
			case One:
				fakeZero = rotations * fullRotationTicks + stepTicks;
				switch (targetPosition) {
					case Zero:
						rotator.setTargetPosition(fakeZero - stepTicks);
						break;
					case One:
						break;
					case Two:
						rotator.setTargetPosition(fakeZero + stepTicks);
						break;
				}
				break;
			case Two:
				fakeZero = rotations * fullRotationTicks + stepTicks * 2;
				switch (targetPosition) {
					case Zero:
						rotator.setTargetPosition(fakeZero + stepTicks);
						break;
					case One:
						rotator.setTargetPosition(fakeZero - stepTicks);
						break;
					case Two:
						break;
				}
				break;
		}

		currentPosition = targetPosition;
	}
}
