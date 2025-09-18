package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 * If you run into any null pointers check your enabled parts of the robot
 */
public class Robot {
	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	public Drivetrain drivetrain;
	public boolean drivetrainEnabled;
	public Intake intake;
	public boolean intakeEnabled;
	//	public IntakeSpinner intakeSpinner;
//	public boolean intakeSpinnerEnabled;
	public Turret.Camera camera;
	public boolean cameraEnabled;
	public Turret turret;
	public boolean turretEnabled;
	public HandsOfGod handsOfGod;
	public boolean handsOfGodEnabled;
	public PalmsOfGod palmsOfGod;
	public boolean palmsOfGodEnabled;

	/**
	 * Do not forget to chain this with all of the enable methods
	 * @param opMode
	 */
	public Robot(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		drivetrain = new Drivetrain(hardwareMap, this.opMode);
		intake = new Intake(this.opMode);
//		intakeSpinner = new IntakeSpinner(this.opMode);
		camera = new Turret.Camera(this.opMode);
		turret = new Turret(this.opMode);
		handsOfGod = new HandsOfGod(this.opMode);
		palmsOfGod = new PalmsOfGod(this.opMode);
	}

	public Robot enablePalmsOfGod() {
		palmsOfGodEnabled = true;
		return this;
	}
	public Robot enableHandsOfGod() {
		handsOfGodEnabled = true;
		return this;
	}
	public Robot enableDriveTrain() {
		drivetrainEnabled = true;
		return this;
	}

	public Robot enableTurret() {
		turretEnabled = true;
		return this;
	}

	public Robot enableIntake() {
		intakeEnabled = true;
		return this;
	}

//	public Robot enableIntakeSpinner() {
//		intakeSpinnerEnabled = true;
//		return this;
//	}

	public Robot enableCamera() {
		cameraEnabled = true;
		return this;
	}

	public void init() {
		if (intakeEnabled)
			intake.init();
//		if (intakeSpinnerEnabled)
//			intakeSpinner.init();
		if (cameraEnabled)
			camera.init();
		if (turretEnabled)
			turret.init();
		if (handsOfGodEnabled)
			handsOfGod.init();
		if (palmsOfGodEnabled)
			palmsOfGod.init();
	}

	public void loop() {
		if (intakeEnabled)
			intake.loop();
		if (cameraEnabled)
			camera.loop();
		if (turretEnabled)
			turret.loop();
		if (handsOfGodEnabled)
			handsOfGod.loop();

	}
}

