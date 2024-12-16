package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorHandler extends DcMotorImpl {
	private DcMotor motor;

	int max = Integer.MAX_VALUE;
	int min = Integer.MIN_VALUE;

	public MotorHandler(DcMotor motor) {
		this.motor = motor;
	}

	public MotorHandler addBounds(int max, int min) {
		this.min = min;
		this.max = max;
		return this;
	}

	public MotorHandler addUpperBound(int max) {
		return this.addBounds(max, min);
	}

	public MotorHandler addLowerBound(int min) {
		return this.addBounds(max, min);
	}

	public void resetBounds() {
		max = Integer.MAX_VALUE;
		min = Integer.MIN_VALUE;
	}
}
