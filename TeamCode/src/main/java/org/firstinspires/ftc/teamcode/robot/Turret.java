package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

//Oh boy
public class Turret {
	DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	DcMotor spinner0, spinner1;
	Servo angle;
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	Gamepad gamepad;
	TouchSensor limit;
	int maxAbsDelta = 2000;
	double rotatorPower = 0;
	double rotation = 0;
	double lastTime = .05;
	double ticksPerRotation = 2000.0 / (2.0 * Math.PI);
	double timeOfLastUpdate = 0;
	double deltaTimeOfLastUpdate = 0;
	boolean useGamepad;
	boolean trackTarget;
	boolean autoTrack = true;
	MecanumDrive mecanumDrive;
	PID rotationPID;
	TurretPose2d pose;
	Vector2d robotMovement = null;
	Vector2d target = new Vector2d(0, 0);
	Vector2d adjustedTarget;

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
		spinner0 = hardwareMap.dcMotor.get("spinny0");
		spinner1 = hardwareMap.dcMotor.get("spinny1");
		angle = hardwareMap.servo.get("turretAngle");


		// If it aint broke dont fix it
		rotationPID = new PID(.1, 0, 0, .005);
	}

	public Turret updatePose(Pose2d pose) {
		if (robotMovement == null) {
			robotMovement = new Vector2d(pose.position.x, pose.position.y);
			deltaTimeOfLastUpdate = 0;
			timeOfLastUpdate = opMode.getRuntime();
		} else {
			// Vector from last position to this one
			double xdiff = pose.position.x - this.pose.pose2d.position.x;
			double ydiff = pose.position.y - this.pose.pose2d.position.y;
			deltaTimeOfLastUpdate = opMode.getRuntime() - timeOfLastUpdate;
			robotMovement = new Vector2d(xdiff, ydiff).div(deltaTimeOfLastUpdate);
			timeOfLastUpdate = opMode.getRuntime();
		}
		double deltaTime = opMode.getRuntime() - timeOfLastUpdate;
		deltaTime = deltaTime * 10;
		adjustedTarget = target.minus(new Vector2d(robotMovement.x * deltaTime, robotMovement.y * deltaTime));
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
			spinner0.setPower(gamepad.left_trigger * ((gamepad.x) ? -1 : 1));
			spinner1.setPower(gamepad.left_trigger * ((gamepad.x) ? -1 : 1));
			angle.setPosition((gamepad.left_stick_y + 1) / 2.0);
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

			double targetRotation = pose.getTurretAngleToTargetRelativeToRobot(adjustedTarget);
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
				Turret.this.mecanumDrive = mecanumDrive;
				updatePose(mecanumDrive.localizer.getPose());
				if (target == null) {
					throw new RuntimeException("No target, please set it");
				}
				if (pose == null) {
					throw new RuntimeException("No pose, please set it");
				}

				rotation = rotator.getCurrentPosition() / ticksPerRotation;
				pose = new TurretPose2d(pose.pose2d, rotation);

				double targetRotation = pose.getTurretAngleToTargetRelativeToRobot(adjustedTarget);
				double currentRotation = pose.rotation;
				double error = rotationPID.calculate(targetRotation - currentRotation, opMode.time - lastTime);
				telemetry.addLine(String.format("deltar: %.3f, dt: %.3f", targetRotation - currentRotation, opMode.time - lastTime));
				lastTime = opMode.getRuntime();
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

	public Action shoot() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				// TODO! do this
				return false;
			}
		};
	}

	double turretHeight = 12, goalHeight = 38.75, a = -386.22; // in/s^2

	/*
	 * Thank you Mr. Cousineau, KVD, and the great tomeng70
	 */
	public void updateTurretAngles() {
		double viy, vix, dx, dy, t, vi, angle;
		dy = goalHeight - turretHeight;

		viy = sqrt(-2 * a * dy); // From CAE's
		t = -viy / a; // vfy is 0
		dx = sqrt(pow(pose.pose2d.position.x - target.x, 2) + pow(pose.pose2d.position.y - target.y, 2)); // x distance not taking movement into account
		vix = dx / t;

		vi = sqrt(pow(vix, 2) + pow(viy, 2));
		angle = atan(viy / vix);
	}
}
