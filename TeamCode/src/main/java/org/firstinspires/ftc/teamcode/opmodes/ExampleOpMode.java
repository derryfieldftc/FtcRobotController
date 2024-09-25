package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ExamplePlugin;

/*
 * This is an example OpMode, it should give basic structure to how an OpMode is formatted and
 * interacts with plugins. The plugin used is the ExamplePlugin which should print "Hello Plugins!"
 * then become a basic tank drive
 */

// Name and group of OpMode
@TeleOp(name = "ExampleOpMode", group = OpModeGroups.SAMPLES)
// All OpModes extend the PluginOpMode class, which enables the functionality of plugins
public class ExampleOpMode extends PluginOpMode {

	// OpModes should only have one function, initPlugins(). It returns an Array of RobotPlugins
	@Override // @Override denotes we are overriding the default function, not strictly needed but is convention
	public RobotPlugin[] initPlugins() {
		// This is where we put our OpModes
		ExamplePlugin examplePlugin = new ExamplePlugin(this, "Hello, world!"); // Instantiating our plugin

		// If we wanted to add another plugin, we would do:
		// OtherPlugin otherPlugin = new OtherPlugin(this);

		//This is the return statement, where we return our Array of RobotPlugins
		return new RobotPlugin[] { examplePlugin };
		// if we were using our other plugin we would say
		// return new RobotPlugin[] { examplePlugin, otherPlugin };
	}
}

//And now you are done!
//This is the whole OpMode!