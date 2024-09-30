package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is the building blocks of every autonomous task
 * A task is one function that an autonomous OpMode performs
 */
public abstract class RobotTask {
	public void init() {}
	public void init_loop() {}
	public void run() {}
	public void stop() {}
}
