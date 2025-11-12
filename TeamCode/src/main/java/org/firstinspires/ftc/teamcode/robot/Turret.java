package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

//Oh boy
public class Turret extends RobotPart {
	public DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	DcMotor spinner0;
	Gamepad gamepad;
	TouchSensor limit;
	public double rotationTrim;
	double rotatorPower = 0;
	double rotation = 0;
	double ticksPerRotation = 2000.0 / (2.0 * Math.PI);
	public boolean refreshEncoder = true;
	boolean useGamepad;
	public boolean trackTarget;
	boolean targetSet = false;
	PID rotationPID;
	TurretPose2d pose;
	Vector target = new Vector(0, 0);

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
		rotator = hardwareMap.dcMotor.get(Part.TurretRotator.name);
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
		spinner0 = hardwareMap.dcMotor.get(Part.LaunchMotor.name);
		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


		// If it aint broke dont fix it
		rotationPID = new PID(.1, 0, 0, .005);
	}

	public Turret setTarget(Vector target) {
		this.target = target;
		targetSet = true;
		return this;
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
		}
		//TODO! tracking... again
	}

	public Turret setSpeed(double speed) {
		spinner0.setPower(speed);
		return this;
	}


	/**
	 * This is action should never finish until the stopAutoTracking Action is called
	 */
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
		dx = sqrt(pow(pose.pose.x - target.x, 2) + pow(pose.pose.y - target.y, 2)); // x distance not taking movement into account
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
			//TODO! fix localizer with pedro

			writer.println(String.format("%f %f %f %f", pose.pose.x, pose.pose.y, pose.pose.theta, pose.rotation));
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
			//TODO! fix this method
			return null;
		} catch (Exception ignored) {throw new Exception(ignored);}
	};
}
