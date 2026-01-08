package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;
import static com.qualcomm.robotcore.util.RobotLog.d;
import static com.qualcomm.robotcore.util.RobotLog.i;
import static com.qualcomm.robotcore.util.RobotLog.w;

import static org.firstinspires.ftc.teamcode.robot.RobotPart.Part.IndicatorLightTurret;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.autonmous.actions.Action;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

public class Turret extends RobotPart {
	//call it a radius of 6in
	public DcMotor rotator;
	public DcMotorEx spinner0;
	public double rotationTrim;
	/**
	 * Not meant to be mutated, the distance in ticks from straight ahead to the starting position of the turret
	 */
	public double rotationInitalOffset = 0;
	double ticksPerRotation = 2000.0; //25 to 95 ratio, 1 full rotation is 2k steps
	public boolean refreshEncoder = true;
	TrackingState tracking;
	TurretPose pose;
	double rotation;
	double targetPower;
	private double targetRotation;
	IndicatorLight light;

	@Configurable
	public static class TurretConfigs {
		static double rotationLimit = PI / 2;

		public static double spinnerP = 0;
		public static double spinnerI = 0;
		public static double spinnerD = 0;
		public static double spinnerF = 0;

		@Configurable
		public static class DistanceToPowerCoefficients {
			public static double m = 0.001714191;// 0.0017258;
			public static double b = 0.276731;//0.315965;
		}
		public static double distanceToPower(double distance) {
			d("AHM DISTANCE %f", distance);
			return DistanceToPowerCoefficients.m * distance + DistanceToPowerCoefficients.b;
		}
	}

	public Turret(OpMode opMode) {
		this(opMode, new TurretPose(new Pose(0, 0, 0), 0));
	}

	public void setPose(TurretPose pose) {
		this.pose = pose;
	}

	// oooh wow look a state machine-ish
	private enum TrackingState {
		STOPPING,
		TRACKING,
		NOT_TRACKING
	}

	@Deprecated
	public enum SpeedByDistance {
		Max (1),
		None (0),
		Close (.44),
		Far (.52);
		public final double power;

		SpeedByDistance(double power) {this.power = power;};
	}

	public Turret(OpMode opMode, TurretPose turretPose2d) {
		super(opMode);
		rotationInitalOffset = (turretPose2d.rotation) * ticksPerRotation;
		pose = turretPose2d;

		rotator = hardwareMap.dcMotor.get(Part.TurretRotator.name);
		spinner0 = (DcMotorEx) hardwareMap.get(Part.LaunchMotor.type, Part.LaunchMotor.name);
		light = new IndicatorLight(opMode, IndicatorLightTurret.name);

		rotator.setPower(0);
		rotator.setTargetPosition(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public Turret setAngleTrim(double rotationTrim) {
		this.rotationTrim = rotationTrim;
		return this;
	}

	public void setRotationPower(double power) {
		rotator.setPower(power);
	}

	public Turret setSpeed(double speed) {
		targetPower = speed;
		spinner0.setPower(speed);
		d("AHM spinner0 power %f", spinner0.getPower());
		return this;
	}

	/*
	OLD
	distance velocity
	133.15	1300
	90		1120
	47		950
	105		1180
	129		1280
	y=4.05014x+757.62475
	NEW
	93		1000
	141		1300
	56.5	820
	107		1180
	 */

	public double getSpeedByDistance(double distance) {
		return clamp(TurretConfigs.distanceToPower(distance), 0, 1); // found empirically
	}

	public void updateLight() {
		double error = rotation - targetRotation;
		double maxAllowedError = .3;
		d("AHM light error %f", error);

		IndicatorLight.Color color;
		if (error > maxAllowedError) {
			color = IndicatorLight.Color.Orange;
		} else if (error < -maxAllowedError) {
			color = IndicatorLight.Color.Indigo;
		} else {
			color = IndicatorLight.Color.Green;
		}
		light.setColor(color);
	}

	public void dumpTelemetry(Telemetry telemetry) {
		telemetry.addData("X", pose.pose.getX());
		telemetry.addData("Y", pose.pose.getY());
		telemetry.addData("R", pose.pose.getHeading());
		telemetry.addData("T", pose.rotation);
	}

	/**
	 * This is action should never finish until the stopAutoTracking Action is called
	 */
	public Action trackTarget(Vector target, Localizer localizer) {
		return new Action() {
			@Override
			public boolean run() {
				if (tracking == TrackingState.NOT_TRACKING)
					tracking = TrackingState.TRACKING;
				updatePose(localizer.getPose());

				double angleToTarget = atan2(target.getYComponent() - pose.pose.getY(), target.getXComponent() - pose.pose.getX());
				d("AHM atan2 %f", angleToTarget);
				targetRotation = (angleToTarget + rotationTrim - pose.pose.getHeading()) % (2 * PI);
				targetRotation = safeRotationAngle(targetRotation);
				d("AHM angle to target %f", angleToTarget);
				updateRotation(targetRotation);
				RobotLog.d("AHM TRACKING target angle %f", targetRotation);
				updateLight();

				return true;
			}
		};
	}

	/**
	 * Compute distance between robot pose and target
	 * @param target
	 * @return
	 */
	public double getDistance(Vector target) {
		double ydiff = this.pose.pose.getY() - target.getYComponent();
		double xdiff = this.pose.pose.getX() - target.getXComponent();
		double distance = sqrt((xdiff * xdiff) + (ydiff * ydiff));
		d("AHM spinner0 distance %f", distance);
		return distance;
	}

	public void updateRotation(double targetRotation) {
		spinner0.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(TurretConfigs.spinnerP,
				TurretConfigs.spinnerI, TurretConfigs.spinnerD, TurretConfigs.spinnerF));
		this.targetRotation = safeRotationAngle(targetRotation);
		rotator.setTargetPosition((int) (-rotationInitalOffset + ((this.targetRotation / (2 * PI)) * ticksPerRotation)));
		d("AHM target rotation %d", rotator.getTargetPosition());
	}

	private void updatePose(Pose pose) {
		rotation = (((rotator.getCurrentPosition() - rotationInitalOffset) / ticksPerRotation));
		d("AHM ROTATOR TICKS %d", rotator.getCurrentPosition());
		d("AHM rotation %f", rotation);
		this.pose = new TurretPose(pose, rotation);
	}

	private double safeRotationAngle(double rotation) {
		if (rotation > PI) {
			rotation = rotation - (2 * PI);
		} else if (rotation < -PI) {
			rotation = rotation + (2 * PI);
		};
		d("AHM SAFE TRUE %f", rotation);
		return clamp(rotation, -TurretConfigs.rotationLimit, TurretConfigs.rotationLimit);
	}

	@SuppressLint("DefaultLocale")
	public void savePosition() {
		File file = new File("/sdcard/FIRST/lastPose");

		try {
			PrintWriter writer = new PrintWriter(file);
			file.createNewFile();

			writer.println(String.format("%f %f %f %f", pose.pose.getX(), pose.pose.getY(), pose.pose.getHeading(), pose.rotation));
			writer.flush();
			writer.close();
			d("AHM WROTE");

		} catch (Exception ignored) {
			d("AHM WRITE FAIL " + ignored);
			throw new RuntimeException(ignored);
		} // beautiful exception handleing
	}

	public static TurretPose getSavedPosition() throws Exception {
		try {
			File file = new File("/sdcard/FIRST/lastPose");
			Scanner scanner = new Scanner(file);
			double x = scanner.nextDouble();
			double y = scanner.nextDouble();
			double r = scanner.nextDouble();
			double t = scanner.nextDouble();
			return new TurretPose(new Pose(x, y, r), t);
		} catch (Exception ignored) {throw new Exception(ignored);}
	}
}
//
//		tagMatch = false;
//		if (ll.getResults() != null && ll.getResults().isValid() && !ll.getResults()
//				.getFiducialResults().isEmpty()) {
//
//			d("AHM got ll results, size: " + ll.getResults().getFiducialResults().size());
//			LLResult llr = ll.getResults();
//
//			if (!gamepad2.start) {
//				for (LLResultTypes.FiducialResult result : llr.getFiducialResults()) {
//					d("AHM tag number " + result.getFiducialId());
//					if (result.getFiducialId() == targetTag.id) {
//						d("AHM matches target tag");
//						telemetry.addData("tx", result.getTargetXDegrees());
//						double tx = -result.getTargetXDegrees();
//						d("AHM tx " + tx);
//						bot.turret.rotator.setPower(tx / 50 * ((gamepad2.start) ? 0 : 1));
//						d("AHM power " + tx / 50);
//						tagMatch = true;
//					}
//				}
//			}
//		}
//
//		if (!tagMatch || gamepad2.start)
//			bot.turret.rotator.setPower(gamepad2.left_stick_x);
//
//		if (tagMatch) {
//			gamepad2.setLedColor(0, 255, 0, 300);
//		} else {
//			gamepad2.setLedColor(255, 0, 0, 300);
//		}
