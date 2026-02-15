package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Lift.Position.Down;
import static org.firstinspires.ftc.teamcode.robot.Lift.Position.Up;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

public class Lift extends RobotPart {
	Servo lift;
	TouchSensor liftSwitch;

	public Lift(OpMode opMode) {
		super(opMode);
		lift = hardwareMap.servo.get(Part.LiftServo.name);
		liftSwitch = hardwareMap.get(TouchSensor.class, "liftSwitch");
	}

	public enum Position {
		Up		(1),
		Down	(0);

		final double position;
		Position(double position) {
			this.position = position;
		}
	}

	public void setPosition(Position position) {
		lift.setPosition(position.position);
	}
	public Position getPosition(){
		return liftSwitch.isPressed() ? Down : Up;
	}
	public boolean isDown(){
		return getPosition() == Down;
	}
}
