package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CenterStageRobot implements iRobot{
    private final LinearOpMode creator;
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;

    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;

    public CenterStageRobot(LinearOpMode creator) {
        this.creator = creator;
        this.hardwareMap = creator.hardwareMap;
        this.telemetry = creator.telemetry;
    }
    @Override
    public void initHardware() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");
    }

    @Override
    public void drive(double distance) {

    }

    @Override
    public void strafe(double distance) {

    }

    @Override
    public void rotate(double degrees) {

    }

    @Override
    public void driveXYRB(double x, double y, double r, double b, double bd) {

    }

    @Override
    public void driveTank(double l, double r, double b) {

    }

    @Override
    public double getIMUHeading() {
        return 0;
    }

    @Override
    public void driveStop() {

    }

    @Override
    public void setMotorMode(DcMotor.RunMode mode) {

    }

    @Override
    public void stopAll() {

    }

    @Override
    public double normalizeHeading(double heading) {
        return 0;
    }
}
