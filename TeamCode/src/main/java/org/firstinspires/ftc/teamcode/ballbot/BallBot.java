package org.firstinspires.ftc.teamcode.ballbot;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.binarybot.BinaryBot.State.SPECIMEN_BACK_OFF;
import static org.firstinspires.ftc.teamcode.binarybot.BinaryBot.State.SPECIMEN_HOOKING;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.binarybot.Manipulator;

public class BallBot {
    // ******************************************************************
    // enumerations.
    // ******************************************************************

    // ******************************************************************
    // Constants
    // ******************************************************************

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    RevHubOrientationOnRobot orientationOnRobot;

    // op mode related items.
    private OpMode opMode;
    private HardwareMap hardwareMap;

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private double previousAngle = 0;
    public double integratedAngle = 0;

    private final boolean USE_AUTO_CORRECT = true;
    private final double P_COEFFICIENT_DRIVE = 0.1;
    private final double P_COEFFICIENT_STRAFE = 0.05;

    public BallBot(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        initHardware();
    }

    private void initHardware() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        initIMU();
    }

    private void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void updateAngles() {
        float currAngle;
        double deltaAngle;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = angles.firstAngle;

        // the binary bot team uses the convention that a positive angle is a clockwise rotation.
        // we need to flip the currAngle to work with this convention.
        currAngle = - currAngle;

        deltaAngle = currAngle - previousAngle;
        if (deltaAngle < -180) {
            // Went from +180 to -180 (direction).
            deltaAngle = deltaAngle + 360;
        } else if (deltaAngle > 180) {
            // Went from -180 to +180 (direction).
            deltaAngle = deltaAngle - 360;
        }
        integratedAngle = integratedAngle + deltaAngle;
        previousAngle = currAngle;
    }

    public void resetAngles() {
        integratedAngle = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        previousAngle = angles.firstAngle;

        // the binary bot team uses the convention that a positive angle is a clockwise rotation.
        // we need to flip the currAngle to work with this convention.
        previousAngle = - previousAngle;
    }

    public double getCurrentAngle() {
        return integratedAngle;
    }

    public double getPreviousAngle() {
        return previousAngle;
    }

    // ******************************************************************
    // public methods
    // ******************************************************************
    /**
     * mecanum drive function.
     *
     * @param drive forward / backward (negative) power
     * @param strafe rightwards / leftwards (negative) power
     * @param twist clockwise / counterclockwise (negative) power
     */
    public void drive(double drive,  double strafe, double twist) {
        double powerFL = drive + strafe + twist;
        double powerBL = drive - strafe + twist;
        double powerFR = drive - strafe - twist;
        double powerBR = drive + strafe - twist;

        powerFL = clamp(1, -1, powerFL);
        powerBL = clamp(1, -1, powerBL);
        powerFR = clamp(1, -1, powerFR);
        powerBR = clamp(1, -1, powerBR);

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    /**
     * Stop the robot.
     */
    public void stop() {
        drive(0, 0, 0);
    }

}