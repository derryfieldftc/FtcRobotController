package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 */
public class Robot {
	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	public Drivetrain drivetrain;
	public Intake intake;
	public IntakeSpinner intakeSpinner;
	public Camera camera;

	public Robot(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		drivetrain = new Drivetrain(hardwareMap, this.opMode);
		intake = new Intake(this.opMode);
		intakeSpinner = new IntakeSpinner(this.opMode);
		camera = new Camera(this.opMode);
	}

	public void init() {
		intake.init();
		intakeSpinner.init();
		camera.init();
	}

	public void loop() {
		intake.loop();
		camera.loop();
	}
}
