package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.ThreadPool;

import java.util.ArrayList;

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
