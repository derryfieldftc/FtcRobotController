package org.firstinspires.ftc.teamcode.binarybot;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class BinaryBot {
    // ******************************************************************
    // enumerations.
    // ******************************************************************
    // odometry system state.
    public enum MeasuredState {
        IDLE,
        FORWARD,
        BACKWARD,
        RIGHTWARD,
        LEFTWARD,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    // ******************************************************************
    // Constants
    // ******************************************************************
    // Configured the two motor ports as Tetrix motors so I can specify COUNTS_PER_MOTOR_REV.
    // by default, Tetrix motors (with 60:1 gearboxes) have 1440 encoder pulses per rev.
    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 2;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    DcMotor driveEncoder;
    DcMotor strafeEncoder;
    IMU imu = null;
    RevHubOrientationOnRobot orientationOnRobot;

    // op mode related items.
    private OpMode opMode;
    private HardwareMap hardwareMap;

    // ******************************************************************
    // public member variables.
    // ******************************************************************
    // ITD arm.
    // make it publicly visible.
    public Manipulator manipulator;

    // current odometry state.
    public MeasuredState measuredState = MeasuredState.IDLE;

    // ******************************************************************
    // construction
    // ******************************************************************
    public BinaryBot(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        initHardware();
    }

    // ******************************************************************
    // private / helper methods.
    // ******************************************************************
    /**
     * Init robot hardware.
     */
    private void initHardware() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

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

        // init the arm.
        manipulator = new Manipulator(hardwareMap, opMode);

        // odometry.
        driveEncoder = hardwareMap.get(DcMotor.class, "drive");
        strafeEncoder = hardwareMap.get(DcMotor.class, "strafe");
        imu = hardwareMap.get(IMU.class, "imu");

        // set orientation on our robot.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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

        // set state to idle.
        measuredState = MeasuredState.IDLE;
    }

    // drive a specified distance in inches.
    public int currPos = 0;
    public int initPos = 0;
    public int tgtPos = 0;
    public float measuredPower = 0;


    /**
     * drive forward/backwards using odometery a measured distance.
     * use negative distance to go backwards.
     *
     * @param power - forward drive power.
     * @param distance - distance to drive (inches).
     */
    public void measuredDrive(double power, double distance)  {

        // get init distance.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = driveEncoder.getCurrentPosition();

        // calculate target position.
        // number of wheel rotations.
        double wheelRev = distance / WHEEL_DIAMETER_INCHES;

        // offset in encoder ticks.
        double offset = wheelRev / COUNTS_PER_MOTOR_REV;
        tgtPos = (int)offset + initPos;
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            measuredState = MeasuredState.BACKWARD;
        } else {
            measuredPower = (float)Math.abs(power);
            measuredState = MeasuredState.FORWARD;
        }
    }

    public void measuredStrafe(double power, double distance)  {

        // get init distance.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = strafeEncoder.getCurrentPosition();

        // calculate target position.
        // number of wheel rotations.
        double wheelRev = distance / WHEEL_DIAMETER_INCHES;

        // offset in encoder ticks.
        double offset = wheelRev / COUNTS_PER_MOTOR_REV;
        tgtPos   = (int)offset + tgtPos;
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            measuredState = MeasuredState.LEFTWARD;
        } else {
            measuredPower = (float)Math.abs(power);
            measuredState = MeasuredState.RIGHTWARD;
        }
    }

    public void measuredTurn(double power, double angle)  {

//        // get init distance.
//        // NOTE: we assume the encoder will not rollover (which it shouldn't)
//        // and don't bother to check for this condition.
//        initStrafePos = strafeEncoder.getCurrentPosition();
//
//        // calculate target position.
//        // number of wheel rotations.
//        double wheelRev = angle / WHEEL_DIAMETER_INCHES;
//
//        // offset in encoder ticks.
//        double offset = wheelRev / COUNTS_PER_MOTOR_REV;
//        tgtStrafePos = (int)offset + initStrafePos;
//        if (offset < 0) {
//            strafePower = -(float)Math.abs(power);
//            measuredState = MeasuredState.LEFTWARD;
//        } else {
//            strafePower = (float)Math.abs(power);
//            measuredState = MeasuredState.RIGHTWARD;
//        }
    }

    /**
     *
     * @return true if still in measured mode.
     */
    public boolean measuredUpdate() {
        switch(measuredState) {
            case IDLE:
                // done.
                return false;
            case FORWARD:
                // update current position
                currPos = driveEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos > tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(measuredPower, 0, 0);
                    return true;
                }
            case BACKWARD:
                // update current position
                currPos = driveEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos < tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(measuredPower, 0, 0);
                    return true;
                }
            case RIGHTWARD:
                // update current position
                currPos = strafeEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos > tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(0, measuredPower, 0);
                    return true;
                }
            case LEFTWARD:
                // update current position
                currPos = strafeEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos < tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(0, measuredPower, 0);
                    return true;
                }
            default:
                return false;
        }
    }
}
