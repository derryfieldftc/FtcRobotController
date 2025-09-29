package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

//Oh boy
public class Turret {
	DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	Gamepad gamepad;
	TouchSensor limit;
	int maxAbsDelta = 2000;
	double rotatorPower = 0;
	double rotation = 0;
	double lastTime;
	double ticksPerRotation = 2000.0 / (2.0 * Math.PI);
	boolean useGamepad;
	boolean trackTarget;
	boolean autoTrack = true;
	PID rotationPID;
	TurretPose2d pose;
	Vector2d target = new Vector2d(0, 0);

	public Turret(OpMode opMode, TurretPose2d turretPose2d) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		gamepad = opMode.gamepad2;
		pose = turretPose2d;
	}

	public Turret useGamepad() {
		useGamepad = true;
		return this;
	}

	public Turret trackTarget() {
		trackTarget = true;
		return this;
	}

	public void init() {
		rotator = hardwareMap.dcMotor.get("turretRotator");
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		limit = hardwareMap.touchSensor.get("turretLimit");
		rotator.setPower(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// If it aint broke dont fix it
		rotationPID = new PID(.1, .001, 0, .005);
	}

	public Turret updatePose(Pose2d pose) {
		this.pose = new TurretPose2d(pose, this.pose.rotation);
		return this;
	}

	public Turret setTarget(Vector2d target) {
		this.target = target;
		return this;
	}

	public void setRotatorPower(double power) {
		rotatorPower = power;
	}

	public void loop() {
		if (useGamepad) {
			rotator.setPower(gamepad.right_stick_y);
			if (gamepad.y) {
				rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
		}

		if (trackTarget) {
			if (target == null) {
				throw new RuntimeException("No target, please set it");
			}
			if (pose == null) {
				throw new RuntimeException("No pose, please set it");
			}

			rotation = rotator.getCurrentPosition() / ticksPerRotation;
			pose = new TurretPose2d(pose.pose2d, rotation);

			double targetRotation = pose.getTurretAngleToTargetRelativeToRobot(target);
			double currentRotation = pose.rotation;
			double error = rotationPID.calculate(targetRotation - currentRotation, opMode.time - lastTime);
			lastTime = opMode.time;
			telemetry.addData("target", targetRotation);
			telemetry.addData("current", currentRotation);
			telemetry.addData("error", error);
			rotator.setPower(error * 10);
		}


		telemetry.addData("motorpos", rotator.getCurrentPosition());
		telemetry.addData("limit", limit.getValue());
		telemetry.update();
	}

	/**
	 * This is action should never finish until the stopAutoTracking Action is called
	 */
	public Action autoTracking(MecanumDrive mecanumDrive) {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				updatePose(mecanumDrive.localizer.getPose());
				if (target == null) {
					throw new RuntimeException("No target, please set it");
				}
				if (pose == null) {
					throw new RuntimeException("No pose, please set it");
				}

				rotation = rotator.getCurrentPosition() / ticksPerRotation;
				pose = new TurretPose2d(pose.pose2d, rotation);

				double targetRotation = pose.getTurretAngleToTargetRelativeToRobot(target);
				double currentRotation = pose.rotation;
				double error = rotationPID.calculate(targetRotation - currentRotation, opMode.time - lastTime);
				lastTime = opMode.time;
				telemetry.addData("target", targetRotation);
				telemetry.addData("current", currentRotation);
				telemetry.addData("error", error);
				telemetry.addLine(String.format("x: %.3f, y: %.3f, t: %.3f", pose.pose2d.position.x, pose.pose2d.position.y, pose.pose2d.heading.toDouble()));
				rotator.setPower(error * 10);
				telemetry.update();
				return true;
			}
		};
	}

	public Action stopAutoTracking() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				autoTrack = false;
				return false;
			}
		};
	}
}
