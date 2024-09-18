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

        ThreadPool pool = new ThreadPool();
        for (RobotPlugin plugin : plugins) {
            // pool.addThread(new Thread(() -> {
                String pluginName = plugin.getClass().getSimpleName();
                Telemetry.Line line = telemetry.addLine("init: " + pluginName);
                telemetry.update();
                plugin.init();
                telemetry.removeLine(line);

                line = telemetry.addLine("init_loop: " + pluginName);
                telemetry.update();
                while (opModeInInit()) {
                    plugin.init_loop();
                }
                waitForStart();
                telemetry.removeLine(line);

                line = telemetry.addLine("start: " + pluginName);
                telemetry.update();
                plugin.start();
                telemetry.removeLine(line);

                while (opModeIsActive()) {
                    plugin.loop();
                    telemetry.update();
                }

                line = telemetry.addLine("start: " + pluginName);
                telemetry.update();
                plugin.stop();
                telemetry.removeLine(line);

                telemetry.update();
            // }));
        }


    }

}
