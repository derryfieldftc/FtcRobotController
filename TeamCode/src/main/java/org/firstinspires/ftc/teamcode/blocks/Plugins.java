package org.firstinspires.ftc.teamcode.blocks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.*;

import java.lang.reflect.Constructor;
import java.util.ArrayList;

public class Plugins extends BlocksOpModeCompanion {
	public static ArrayList<RobotPlugin> plugins;
	@ExportToBlocks
	public static void usePlugin(String name) {
		RobotPlugin addedPlugin;
		try {
			addedPlugin = (RobotPlugin) Class.forName("org.firstinspires.ftc.teamcode.plugins." + name).getConstructor(OpMode.class).newInstance(opMode);;

			plugins.add(addedPlugin);
			addedPlugin.init();
		} catch (Exception e) {
			telemetry.addLine("Uh Oh, The plugin " + name + " was not found or had some other error\n" + e.getMessage());
		}
	}

	@ExportToBlocks
	public static void updatePlugins() {
		for (RobotPlugin plugin : plugins) {
			plugin.loop();
		}
	}
}
