package org.firstinspires.ftc.teamcode.plugins.devices;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;

public interface DeviceHandler {
	void info(DeviceTest.Part part);
	DeviceTest.Part init(HardwareDevice device, String name, HardwareMap hardwareMap);
	void editValues(DeviceTest.Part part, GamepadManager gamepad);
	void updateValues(DeviceTest.Part part);
	void helpMenu();

}
