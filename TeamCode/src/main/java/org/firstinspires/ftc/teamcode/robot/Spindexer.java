package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.util.RobotLog.d;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;

import java.util.Arrays;

/*
 *  0
 * 2 1
 *
 * 0 shooting
 * 1 left
 * 2 right
 */
public class Spindexer extends RobotPart {
	DcMotorEx rotator;
	Servo lift;
	Position currentPosition = Position.Zero;
	TouchSensor limit;
	ColorSensor shooting, left, right;
	IndicatorLight first, second, third;
	Field.Ball[] balls = new Field.Ball[3];
	// 0 1 2

	// 4:1 ratio
	// 103.8 ticks per revolution
	// so to spin large gear its 415.2 ticks
	final int fullRotationTicks = 415;
	final int stepTicks = 138;

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

		rotator = hardwareMap.get(DcMotorEx.class, Part.SpindexerRotator.name);
		resetSpindexer();

		limit = hardwareMap.touchSensor.get(Part.SpindexerLimit.name);

		lift = hardwareMap.servo.get(Part.SpindexerLift.name);

		shooting = hardwareMap.colorSensor.get(Part.SpindexerColor0.name);
		left = hardwareMap.colorSensor.get(Part.SpindexerColor1.name);
		right = hardwareMap.colorSensor.get(Part.SpindexerColor2.name);

		first = new IndicatorLight(hardwareMap.servo.get(Part.IndicatorLight0.name));
		second = new IndicatorLight(hardwareMap.servo.get(Part.IndicatorLight1.name));
		third = new IndicatorLight(hardwareMap.servo.get(Part.IndicatorLight2.name));

		first.setColor(IndicatorLight.Color.Orange);
		second.setColor(IndicatorLight.Color.Green);
		third.setColor(IndicatorLight.Color.Blue);
	}

	public void updateBalls() {
		switch (currentPosition) {
			case Zero:
				balls = new Field.Ball[]{getShootingBall(), getRightBall(), getLeftBall()};
			case One:
				balls = new Field.Ball[]{getLeftBall(), getShootingBall(), getRightBall()};
				break;
			case Two:
				balls = new Field.Ball[]{getRightBall(), getLeftBall(), getShootingBall()};
				break;
		}
		d("AHM balls " + Arrays.toString(balls));
		updateLights();
	}

	private void updateLights() {
		first.setColor(getColorFromBall(balls[0]));
		second.setColor(getColorFromBall(balls[1]));
		third.setColor(getColorFromBall(balls[2]));

	}

	private IndicatorLight.Color getColorFromBall(Field.Ball ball) {
		switch (ball) {
			case Green:
				return IndicatorLight.Color.Green;
			case Purple:
				return IndicatorLight.Color.Violet;
			case Unknown:
				return IndicatorLight.Color.Yellow;
			case None:
				return IndicatorLight.Color.Off;
		}
		return IndicatorLight.Color.Red;
	}

	/**
	 * Returns the ball that is in the shooting position
	 * @return
	 */
	public Field.Ball getShootingBall() {
		float[] temp = new float[3];
		Color.RGBToHSV(shooting.red(), shooting.green(), shooting.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.Zero()); // what the heck, why do enums need constructors I hate I hate I hate
	}

	/**
	 * Returns the ball that is in the left position
	 * @return
	 */
	public Field.Ball getLeftBall() {
		float[] temp = new float[3];
		Color.RGBToHSV(left.red(), left.green(), left.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.One()); // what the heck, why do enums need constructors I hate I hate I hate
	}

	/**
	 * Returns the ball that is in the right position
	 * @return
	 */
	public Field.Ball getRightBall() {
		float[] temp = new float[3];
		Color.RGBToHSV(right.red(), right.green(), right.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.Two()); // what the heck, why do enums need constructors I hate I hate I hate
	}

	private void resetSpindexer() {
		rotator.setTargetPosition(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rotator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
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

	public boolean touchSensorPressed() {
		return limit.isPressed();
	}

	public Action resetPosition() {
		return new Action() {
			@Override
			public boolean run() {
				rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				rotator.setPower(-.1);
				if (touchSensorPressed()) {
					rotator.setPower(0);
					resetSpindexer();
					return false;
				}
				return true;
			}
		};
	}

	public boolean atPosition() {
		return !rotator.isBusy();
	}

	public Action waitUntilFinished() {
		return new Action() {
			@Override
			public boolean run() {
				return atPosition();
			}
		};
	}

	/**
	 * Spins to whichever position
	 * @param targetPosition
	 */
	public void setPosition(Position targetPosition) {
		int currentPositionTicks = rotator.getCurrentPosition();
		int rotations = (currentPositionTicks + (stepTicks / 2)) / fullRotationTicks;

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

		d("AHM spin position " + rotator.getCurrentPosition());
		d("AHM spin target " + rotator.getTargetPosition());

		currentPosition = targetPosition;
	}
}
