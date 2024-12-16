package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.AnyTask;

@Autonomous(name = "AnyTask test", group = OpModeGroups.AUTO)
public class AnyTaskTest extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new AnyTask(hardwareMap.servo.get("claw"), (c) -> { Servo C = (Servo) c; C.setPosition(1);}),
				new AnyTask().motor(hardwareMap.dcMotor.get("slide"), (m) -> {m.setTargetPosition(100);}),
				new AnyTask().servo(hardwareMap.servo.get("claw"), (s) -> {s.setPosition(0);}),
		};
	}
}
