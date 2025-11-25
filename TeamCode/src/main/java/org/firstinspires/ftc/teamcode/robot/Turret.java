package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;
import static com.qualcomm.robotcore.util.RobotLog.d;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import android.annotation.SuppressLint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

//Oh boy
public class Turret extends RobotPart {
	//call it a radius of 6in
	public DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	DcMotor spinner0;
	Gamepad gamepad;
	TouchSensor limit;
	public double rotationTrim;
	double ticksPerRotation = 2000.0;
	public boolean refreshEncoder = true;
	TrackingState tracking;
	PID rotationPID;
	TurretPose pose;
	double rotation;
	private double targetAngle;

	// oooh wow look a state machine-ish
	private enum TrackingState {
		STOPPING,
		TRACKING,
		NOT_TRACKING
	}

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

	public void init() {
		rotator = hardwareMap.dcMotor.get(Part.TurretRotator.name);
		limit = hardwareMap.touchSensor.get("turretLimit");
		rotator.setPower(0);
		rotator.setTargetPosition(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		spinner0 = hardwareMap.dcMotor.get(Part.LaunchMotor.name);
		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// If it aint broke dont fix it
		rotationPID = new PID(.1, 0, 0, .005);
	}

	public Turret setSpeed(double speed) {
		spinner0.setPower(speed);
		return this;
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

				double angleToTarget = atan2(pose.pose.getY() - target.getYComponent(), pose.pose.getX() - target.getXComponent());
				RobotLog.d("AHM TRACKING target angle %f", angleToTarget);
				targetAngle = angleToTarget + rotationTrim;

				return tracking == TrackingState.TRACKING;
			}
		};
	}

	public void updateRotation(double targetRotation) {
		rotator.setTargetPosition((int) (targetRotation * ticksPerRotation));
	}

	private void updatePose(Pose pose) {
		rotation = (rotation + (rotator.getCurrentPosition() / ticksPerRotation)) % (2 * PI);
		this.pose = new TurretPose(pose, rotation);
	}

	@SuppressLint("DefaultLocale")
	public void savePosition() {
		File file = new File("/sdcard/FIRST/lastPose");

		try {
			PrintWriter writer = new PrintWriter(file);
			file.createNewFile();
			//TODO! fix localizer with pedro

			writer.println(String.format("%f %f %f %f", pose.pose.getX(), pose.pose.getY(), pose.pose.getHeading(), pose.rotation));
			writer.flush();
			writer.close();

		} catch (Exception ignored) {
			throw new RuntimeException(ignored);
		} // beautiful exception handleing
	}

	public Action trackTag(LimeLight ll, Tag target) {
		return new Action() {
			@Override
			public boolean run() {
				if (tracking == TrackingState.NOT_TRACKING)
					tracking = TrackingState.TRACKING;
				RobotLog.d("AHM ALJKHSGFSLKJDJLGKSJLKJGLK");
				if (ll.getResults() != null && ll.getResults().isValid() && !ll.getResults()
						.getFiducialResults().isEmpty()) {

					d("AHM got ll results, size: " + ll.getResults().getFiducialResults().size());
					LLResult llr = ll.getResults();

					for (LLResultTypes.FiducialResult result : llr.getFiducialResults()) {
						d("AHM tag number " + result.getFiducialId());
						if (result.getFiducialId() == target.id) {
							d("AHM matches target tag");
							telemetry.addData("tx", result.getTargetXDegrees());
							double tx = -result.getTargetXDegrees();
							d("AHM tx " + tx);
							rotator.setPower(tx / 50);
							d("AHM power " + tx / 50);
						}
					}
				}
				return tracking == TrackingState.TRACKING;
			}
		};
	}

	public static TurretPose getSavedPosition() throws Exception {
		try {
			File file = new File("/sdcard/FIRST/lastPose");
			Scanner scanner = new Scanner(file);
			//TODO! fix this method
			return null;
		} catch (Exception ignored) {throw new Exception(ignored);}
	};
}
