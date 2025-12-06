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
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonmous.actions.Action;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

//Oh boy
public class Turret extends RobotPart {
	//call it a radius of 6in
	public DcMotor rotator;
	public DcMotorEx spinner0;
	Gamepad gamepad;
	public double rotationTrim;
	double ticksPerRotation = 2000.0; //25 to 95 ratio, 1 full rotation is 2k steps
	public boolean refreshEncoder = true;
	TrackingState tracking;
	PID rotationPID;
	TurretPose pose;
	double rotation;
	double targetPower;
	private double targetRotation;
	IndicatorLight light;

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
		gamepad = opMode.gamepad2;
		pose = turretPose2d;
	}
	public Turret setAngleTrim(double rotationTrim) {
		this.rotationTrim = rotationTrim;
		return this;
	}

	public void setRotationPower(double power) {
		rotator.setPower(power);
	}

	public void init() {
		rotator = hardwareMap.dcMotor.get(Part.TurretRotator.name);
		rotator.setPower(0);
		rotator.setTargetPosition(0);
		if (refreshEncoder) {
			rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
		spinner0 = (DcMotorEx) hardwareMap.get(Part.LaunchMotor.type, Part.LaunchMotor.name);
		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		light = new IndicatorLight(opMode, IndicatorLightTurret.name);

		// If it aint broke dont fix it
		rotationPID = new PID(.1, 0, 0, .005);
	}

	public Turret setSpeed(double speed) {
		targetPower = speed;
		spinner0.setPower(speed);
		return this;
	}

	public double getSpeedByDistance(double distance) {
		return clamp(0.0017258 * distance + 0.315965, 0, 1); // found empirically
	}

	public void updateLight() {
		double error = rotation - targetRotation;
		double maxAllowedError = .7;
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
				targetRotation = (angleToTarget + rotationTrim - pose.pose.getHeading()) % (PI * 2);
				updateRotation(targetRotation);
				RobotLog.d("AHM TRACKING target angle %f", targetRotation);
				updateLight();

				return tracking == TrackingState.TRACKING;
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
		return sqrt((xdiff * xdiff) + (ydiff * ydiff));
	}

	public void updateRotation(double targetRotation) {
		this.targetRotation = targetRotation % (2 * PI);
		rotator.setTargetPosition((int) ((this.targetRotation / (2 * PI)) * ticksPerRotation));
	}

	private void updatePose(Pose pose) {
		rotation = ((rotator.getCurrentPosition() / ticksPerRotation)) % (2 * PI);
		d("AHM rotation %f", rotation);
		this.pose = new TurretPose(pose, rotation);
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
	};
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
