package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

//Oh boy
public class Turret extends RobotPart {
	public DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	DcMotor spinner0;
	Servo angle;
	double angleAngle = 0; // max .3
	Gamepad gamepad;
	TouchSensor limit;
	int maxAbsDelta = 2000;
	public double rotationTrim;
	double rotatorPower = 0;
	double rotation = 0;
	double lastTime = .05;
	double ticksPerRotation = 2000.0 / (2.0 * Math.PI);
	public boolean refreshEncoder = true;
	double timeOfLastUpdate = 0;
	double deltaTimeOfLastUpdate = 0;
	double targetAngle;
	boolean useGamepad;
	public boolean trackTarget;
	public boolean autoTrack = true;
	boolean targetSet = false;
	PID rotationPID;
	TurretPose2d pose;
	Vector2d robotMovement = null;
	Vector2d target = new Vector2d(0, 0);
	Vector2d adjustedTarget;

	public enum SpeedByDistance {
		Max (1),
		None (0),
		Close (.44),
		Far (.56);
		public final double power;

		SpeedByDistance(double power) {this.power = power;};
	}

	public Turret(OpMode opMode, TurretPose2d turretPose2d) {
		super(opMode);
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

	public Turret setAngleTrim(double rotationTrim) {
		this.rotationTrim = rotationTrim;
		return this;
	}

	public void init() {
		rotator = hardwareMap.dcMotor.get("turretRotator");
		if (refreshEncoder) {
			rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
		limit = hardwareMap.touchSensor.get("turretLimit");
		rotator.setPower(0);
		if (refreshEncoder) {
			rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
		spinner0 = hardwareMap.dcMotor.get("spinny0");
		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
		targetSet = true;
		return this;
	}

	/**
	 * Changes angle by angle units, clamped to 0-1
	 * @param angle
	 */
	public void tuneAngle(double angle) {
		this.angleAngle = clamp(angleAngle + angle / 30, 0, 1);
	}

	public void setRotatorPower(double power) {
		rotatorPower = power;
	}

	public double getRotation() {
		return this.rotation;
	}

	public void loop() {
		if (useGamepad) {
			rotator.setPower(gamepad.right_stick_y / 5);
			if (gamepad.y) {
				rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
			spinner0.setPower(gamepad.left_trigger * ((gamepad.x) ? -1 : 1));
			angleAngle = clamp(angleAngle + -gamepad.left_stick_y / 30, -1, 1);
			telemetry.addData("Angle", angleAngle);
			angle.setPosition(angleAngle);
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

			double currentRotation = pose.rotation;
			if (targetSet && adjustedTarget != null) {
				targetAngle = pose.getTurretAngleToTargetRelativeToRobot(adjustedTarget);
			} else {
				targetAngle = currentRotation;
			}
			targetAngle += rotationTrim;
			double error = rotationPID.calculate(targetAngle - currentRotation, opMode.getRuntime() - lastTime);
			lastTime = opMode.getRuntime();
			telemetry.addData("target", targetAngle);
			telemetry.addData("current", currentRotation);
			telemetry.addData("error", error);
			rotator.setPower(error * 10);
		}


	}

	public void setTargetAngle(double angle) {
		this.targetAngle = angle;
	};

	public Turret setSpeed(double speed) {
		spinner0.setPower(speed);
		return this;
	}

	public Turret setAngle(double angle) {
		this.angle.setPosition(angle);
		return this;
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
				if (targetRotation > 2 * Math.PI - PI / 6)
					targetRotation = 2 * PI - PI / 6;
				double error = rotationPID.calculate(targetRotation - currentRotation, opMode.getRuntime() - lastTime);
				telemetry.addLine(String.format("deltar: %.3f, dt: %.3f", targetRotation - currentRotation, opMode.time - lastTime));
				lastTime = opMode.getRuntime();
				telemetry.addData("target", targetRotation);
				telemetry.addData("current", currentRotation);
				telemetry.addData("error", error);
				telemetry.addLine(String.format("x: %.3f, y: %.3f, t: %.3f", pose.pose2d.position.x, pose.pose2d.position.y, pose.pose2d.heading.toDouble()));
					rotator.setPower(error * 10);
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
	 *
	 * A brief explanation:
	 * The turret knows where it is at all times. It knows this because it knows where it isn't. By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater), it obtains a difference, or deviation. The guidance subsystem uses deviations to generate corrective commands to drive the turret from a position where it is to a position where it isn't, and arriving at a position where it wasn't, it now is. Consequently, the position where it is, is now the position that it wasn't, and it follows that the position that it was, is now the position that it isn't.
	 * In the event that the position that it is in is not the position that it wasn't, the system has acquired a variation, the variation being the difference between where the turret is, and where it wasn't. If variation is considered to be a significant factor, it too may be corrected by the GEA. However, the turret must also know where it was.
	 * The turret guidance computer scenario works as follows. Because a variation has modified some of the information the turret has obtained, it is not sure just where it is. However, it is sure where it isn't, within reason, and it knows where it was. It now subtracts where it should be from where it wasn't, or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be, and where it was, it is able to obtain the deviation and its variation, which is called error.
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

	@SuppressLint("DefaultLocale")
public void savePosition() {
		File file = new File("/sdcard/FIRST/lastPose");

		try {
			PrintWriter writer = new PrintWriter(file);
			file.createNewFile();
			mecanumDrive.localizer.update();
			updatePose(mecanumDrive.localizer.getPose());

			writer.println(String.format("%f %f %f %f", pose.pose2d.position.x, pose.pose2d.position.y, pose.pose2d.heading.toDouble(), pose.rotation));
			writer.flush();
			writer.close();

		} catch (Exception ignored) {
			throw new RuntimeException(ignored);
		} // beautiful exception handleing
	}

	public static TurretPose2d getSavedPosition() throws Exception {
		try {
			File file = new File("/sdcard/FIRST/lastPose");
			Scanner scanner = new Scanner(file);
			return new TurretPose2d(
					new Pose2d(
					new Vector2d(scanner.nextDouble(),	// x
					scanner.nextDouble()),				// y
					Rotation2d.fromDouble(scanner.nextDouble())),				// r
					scanner.nextDouble()				// t
			);
		} catch (Exception ignored) {throw new Exception(ignored);}
	};
}
