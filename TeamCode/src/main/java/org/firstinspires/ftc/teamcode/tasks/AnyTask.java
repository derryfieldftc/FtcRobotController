package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotTask;

import java.util.function.Consumer;

public class AnyTask extends RobotTask {
	Consumer<HardwareDevice> function;
	HardwareDevice device;

	public AnyTask(HardwareDevice hardwareDevice, Consumer<HardwareDevice> consumer) {
		this.device = hardwareDevice;
		this.function = consumer;
	}

	public AnyTask() {};

	public AnyTask servo(Servo servo, Consumer<Servo> consumer) {
		return new AnyTask() {
			public void run() {
				consumer.accept(servo);
			}
		};
	}

	public AnyTask motor(DcMotor motor, Consumer<DcMotor> consumer) {
		return new AnyTask() {
			public void run() {
				consumer.accept(motor);
			}
		};
	}

	public void run() {
		function.accept(device);
	}
}
