package org.firstinspires.ftc.teamcode.plugin.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;

/*
 * This is an example plugin, all it does is print out "Hello Plugins!", then becomes a basic tank
 * drive. This should be used as a reference when developing future plugins.
 */
public class ExamplePlugin extends RobotPlugin {

	// Instance fields
	String message;
	Telemetry telemetry; // Needed for printing to the screen
	OpMode opMode; // Needed for getting the elapsed time

	/*
	 * This is where we instantiate our plugin.
	 * Generally all plugins take an OpMode as a parameter.
	 * The OpMode is then used to get the different hardware, and telemetry the plugin uses.
	 */
	public ExamplePlugin(OpMode opMode, String message) {
		this.message = message;	// set our message
		this.telemetry = opMode.telemetry;	// set the telemetry
		this.opMode = opMode;
	}

	/*
	 * This is the init function described in the RobotPlugin class.
	 * It is ran once when the driver presses the `init` button.
	 * This example uses it to load our message.
	 */
	@Override
	public void init() {
		telemetry.addLine(message); // Adds our message line to telemetry
	}

	/*
	 * This is the init_loop function, it is described in the RobotPlugin class.
	 * It is ran repeatedly when the driver presses init.
	 * It is not necessary for our program, so we can either leave it blank, or not include it
	 */
	@Override
	public void init_loop() {}

	/*
	 * This is the start function, it is ran once when the driver presses play
	 * it is not needed for this plugin
	 */
	@Override
	public void start() {}

	/*
	 * This is the loop function, it is described in the RobotPlugin class.
	 * It is ran repeatedly during the program.
	 * This is generally the core of each plugin, This is generally where the logic goes and where
	 * control of any servos or motors would go.
	 */
	@Override
	public void loop() {
		telemetry.addData("Runtime", opMode.getRuntime());
	}

	/*
	 * This is the stop function
	 * it can be used to "clean up" after an OpMode has finished running
	 * it is not necessary
	 */
	public void stop() {}
}
