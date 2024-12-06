package org.firstinspires.ftc.teamcode.blocks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.*;

import java.util.ArrayList;

@ExportClassToBlocks
public class Plugins extends BlocksOpModeCompanion {
	private static ArrayList<RobotPlugin> plugins = new ArrayList<>();

	@ExportToBlocks
	public static void usePlugin(String name) {
		RobotPlugin addedPlugin;
		try {
			addedPlugin = (RobotPlugin) Class.forName("org.firstinspires.ftc.teamcode.plugins." + name).getConstructor(OpMode.class).newInstance(opMode);;

			plugins.add(addedPlugin);
			telemetry.addLine("Init: " + name + ((addedPlugin == null) ? "null" : addedPlugin.toString()));
			addedPlugin.init();
		} catch (Exception e) {
			telemetry.addLine("Uh Oh, The plugin " + name + " was not found or had some other error\n" + e.getMessage());
		}
	}

	@ExportToBlocks
	public static void initLoopPlugins() {
		for (RobotPlugin plugin : plugins) {
			plugin.init_loop();
		}
	}

	@ExportToBlocks
	public static void loopPlugins() {
		for (RobotPlugin plugin : plugins) {
			plugin.loop();
		}
	}

	@ExportToBlocks
	public static void startPlugins() {
		for (RobotPlugin plugin : plugins) {
			plugin.start();
		}
	}

	@ExportToBlocks
	public static void stopPlugins() {
		for (RobotPlugin plugin : plugins) {
			plugin.stop();
		}
	}

	@ExportToBlocks
	public static String getPlugins() {
		return plugins.toString();
	}
}
