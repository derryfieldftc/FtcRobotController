package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

/*
 * This is an example plugin, all it does is print out "Hello Plugins!", then becomes a basic tank
 * drive. This should be used as a reference when developing future plugins.
 */
public class ExamplePlugin extends RobotPlugin {

	//Variables
	String message;		//Our message
	Telemetry telemetry;	//Telemetry
	OpMode opMode;		//This is used to get all of the motors, and other pieces of the robot.
	HardwareMap hardwareMap;
	Gamepad gamepad;

	DcMotor motorBL;	//back left motor
	DcMotor motorBR;	//back right motor
	DcMotor motorFR;	//front right motor
	DcMotor motorFL;	//front left motor

	/*
	 * This is where we instantiate our plugin.
	 * Generally all plugins take an OpMode as a parameter.
	 * The OpMode is then used to get the different hardware, and telemetry the plugin uses.
	 */
	public ExamplePlugin(OpMode opMode) {
		this.message = "Hello Plugins!";	//set our message
		this.opMode = opMode;	//set our opMode to opMode
		this.telemetry = opMode.telemetry;	//set the telemetry
		this.hardwareMap = opMode.hardwareMap;	//the hardware map
		this.gamepad = opMode.gamepad1;	//our gamepad

		/*
		 * Motor naming is standardized for the movement motors. The convention is:
		 * Back left motor = motorBL
		 * Back right motor = motorBR
		 * etc.
		 */
		this.motorBL = hardwareMap.dcMotor.get("motorBL");	//this sets motorBL
		this.motorBR = hardwareMap.dcMotor.get("motorBR");	//this sets motorBR
		this.motorFR = hardwareMap.dcMotor.get("motorFR");	//this sets motorFR
		this.motorFL = hardwareMap.dcMotor.get("motorFL");	//this sets motorFL

		/* this sets the motor direction.
		 * in general all motors on the left side are set to reverse
		 */
		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
		motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	/*
	 * This is the init function described in the RobotPlugin class.
	 * It is ran once when the driver presses the `init` button.
	 * This example uses it to load our message.
	 */
	@Override
	public void init() {
		telemetry.addLine(message); //Adds our message line to telemetry
		telemetry.update();	//updates telemetry
	}

	/*
	 * This is the init_loop function, it is described in the RobotPlugin class.
	 * It is ran repeatedly when the driver presses init.
	 * It is not necessary for our program, so we can either leave it blank, or not include it
	 */
	@Override
	public void init_loop() {
		//This function is not needed for this plugin
	}

	/*
	 * This is the start function, it is ran once when the driver presses play
	 * it is not needed for this plugin
	 */
	public void start() {
		//This function is not needed for this plugin
	}

	/*
	 * This is the loop function, it is described in the RobotPlugin class.
	 * It is ran repeatedly during the program.
	 * This is generally the core of each plugin, This is generally where the logic goes and where
	 * control of any servos or motors would go.
	 */
	@Override
	public void loop() {
		double leftSidePower = -gamepad.left_stick_y;	//sets the power for each side
		double rightSidePower = -gamepad.right_stick_y;	//remember that up on the Y direction is negative

		//sets power for all the motors
		motorBL.setPower(leftSidePower);
		motorBR.setPower(rightSidePower);
		motorFL.setPower(leftSidePower);
		motorFR.setPower(rightSidePower);
	}

	/*
	 * This is the stop function
	 * it can be used to "clean up" after an OpMode has finished running
	 * it is not necessary
	 */
	public void stop() {
		//Not needed
	}
}
