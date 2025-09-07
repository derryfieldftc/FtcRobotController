package org.firstinspires.ftc.teamcode.plugin.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;
import org.firstinspires.ftc.teamcode.ThreadPool;

public abstract class PluginOpMode extends LinearOpMode {

    public abstract RobotPlugin[] initPlugins();

    private RobotPlugin[] plugins;

    @Override
    public void runOpMode() {
        this.plugins = initPlugins();


        for (RobotPlugin plugin : plugins) {
			plugin.init();
        }

		while (opModeInInit()) {
			for (RobotPlugin plugin : plugins) {
				plugin.init_loop();
			}
		}

		waitForStart();

		for (RobotPlugin plugin : plugins) {
			plugin.start();
		}

		while (opModeIsActive()) {
			for (RobotPlugin plugin : plugins) {
				plugin.loop();
			}
		}

		for (RobotPlugin plugin : plugins) {
			plugin.stop();
		}


    }

}
