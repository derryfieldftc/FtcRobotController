package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorRobot extends Robot{
    private HardwareMap hardwareMap;

    private DcMotor driveBL;
    private DcMotor driveBR;
    private DcMotor driveFL;
    private DcMotor driveFR;
    private Rev2mDistanceSensor fSensor;
    private Rev2mDistanceSensor lSensor;
    private Rev2mDistanceSensor rSensor;
    private Rev2mDistanceSensor bSensor;

    public enum Sensors {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }
    public SensorRobot(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
//        driveBL = this.hardwareMap.get(DcMotor.class, "motorBL");
//        driveBR = this.hardwareMap.get(DcMotor.class, "motorBR");
//        driveFL = this.hardwareMap.get(DcMotor.class, "motorFL");
//        driveFR = this.hardwareMap.get(DcMotor.class, "motorFR");
        fSensor = this.hardwareMap.get(Rev2mDistanceSensor.class, "front");
        lSensor = this.hardwareMap.get(Rev2mDistanceSensor.class, "left");
        rSensor = this.hardwareMap.get(Rev2mDistanceSensor.class, "back");
        bSensor = this.hardwareMap.get(Rev2mDistanceSensor.class, "right");
//        driveFL.setDirection(DcMotor.Direction.REVERSE);
//        driveBL.setDirection(DcMotor.Direction.REVERSE);
//        driveFR.setDirection(DcMotor.Direction.FORWARD);
//        driveBR.setDirection(DcMotor.Direction.FORWARD);

//        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    //gets distance. On distanceOutOfRange, returns DistanceUnit.infinity
    public double getDistance(Sensors sensor) {
        switch (sensor) {
            case FRONT:
                return fSensor.getDistance(DistanceUnit.METER);
            case BACK:
                return bSensor.getDistance(DistanceUnit.METER);
            case LEFT:
                return lSensor.getDistance(DistanceUnit.METER);
            case RIGHT:
                return rSensor.getDistance(DistanceUnit.METER);
        }
        return DistanceUnit.infinity;
    }
    public void drive(double axial, double lateral, double yaw){
        super.drive(axial, lateral, yaw);
    }
}
