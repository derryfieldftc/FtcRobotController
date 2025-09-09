package org.firstinspires.ftc.teamcode.robot;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {
    // ******************************************************************
    // enumerations.
    // ******************************************************************
    // measured movement system state.
    public enum State {
        IDLE,
        NAVIGATING,
        DELAYING
    }

    public State state = State.IDLE;
    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public Pose pose;
    public Pose waypoint;

    public DSLog log;

    // keep track of previous encoder positions (as encoder counts)
    int prevLeftPos;
    int prevRightPos;
    int prevAuxPos;

    // ******************************************************************
    // Constants
    // ******************************************************************
    // Assume GoBilda 4-Bar Odometry Pod
    // see https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/
    // ******************************************************************
    static final double     POD_COUNTS_PER_REV    = 2000;
    static final double POD_WHEEL_DIAMETER_CM = 3.2;

    static final double POD_WHEEL_CIRCUM_CM = Math.PI * POD_WHEEL_DIAMETER_CM;
    static final double POD_COUNTS_PER_CM = POD_COUNTS_PER_REV / POD_WHEEL_CIRCUM_CM;
    static final double     POD_COUNTS_PER_INCH         =   POD_COUNTS_PER_CM * 2.54;

    // spacing between left and right encoder wheels.
    static final double POD_SPACING_LR_CM = 41.0;

    // spacing from center point to side-to-side encoder wheel.
    static final double POD_SPACING_AUX_CM = 0.0;

    static final double TRACK_WIDTH = 41.5;                 // cm
    static final double WHEELBASE = 33.7;                   // cm

    static final double     DRIVE_COUNTS_PER_REV    = 537.7;
    static final double     DRIVE_DRIVE_GEAR_REDUCTION    = 1;

    // GoBilda new mecanum wheels have 104mm diameter (4.094 inches)
    static final double     DRIVE_WHEEL_DIAMETER_INCHES   = 4.094;

    static final double     DRIVE_WHEEL_RADIUS_CM = DRIVE_WHEEL_DIAMETER_INCHES / 2.0 * 2.54;
    static final double     DRIVE_WHEEL_CIRCUM_INCHES = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;

    static final double     DRIVE_COUNTS_PER_INCH         =   (DRIVE_COUNTS_PER_REV * DRIVE_DRIVE_GEAR_REDUCTION) /
            (DRIVE_WHEEL_CIRCUM_INCHES);

    static final boolean USE_ODOMETRY_POD = true;
    static final double STRAFE_ENCODER_FUDGE_FACTOR = 1.0;

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    // op mode related items.
    private OpMode opMode;
    private HardwareMap hardwareMap;

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private double previousAngle = 0;
    public double integratedAngle = 0;

    public double previous_time = 0;
    public double current_time = 0;
    public double delta_time = 0;
    ElapsedTime elapsed_time;

    PID pid_x;
    PID pid_y;
    PID pid_theta;

    public void setPIDX(PID pid) {
        this.pid_x = pid;
    }

    public PID getPIDX() {
        return pid_x;
    }

    public void setPIDY(PID pid) {
        this.pid_y = pid;
    }

    public PID getPIDY() {
        return pid_y;
    }

    public void setPIDTheta(PID pid) {
        this.pid_theta = pid;
    }

    public PID getPIDTheta() {
        return pid_theta;
    }


    private final boolean USE_AUTO_CORRECT = true;
    private final double P_COEFFICIENT_DRIVE = 0.1;
    private final double P_COEFFICIENT_STRAFE = 0.05;

    public boolean motor_correction_enabled = false;

    public void setMotorCorrectionEnabled(boolean value) {
        this.motor_correction_enabled = value;
    }

    public boolean getMotorCorrectionEnabled() {
        return motor_correction_enabled;
    }

    public Drivetrain(HardwareMap hardwareMap, OpMode opMode) {

        elapsed_time = new ElapsedTime();
        log = new DSLog("/sdcard/FIRST/drivetrain_log.txt");
        log.log("x, y, theta, err_x, err_y, err_theta, err_x_local, err_y_local, power_x_local, power_y_local, power_theta, powerFL, powerBL, powerBR, powerFR");
        pose = new Pose (0,	0,0);


//        pid_x = new PID(0.014, 0.001, 0.000, 0.0001);
//        pid_y = new PID(0.014, 0.001, 0.000, 0.0001);
//        pid_theta = new PID(0.2, 0.07, 0.000, Math.toRadians(0.0001));

        // set default PID values.
        // note user can change this useing the setPID methods.
        pid_x = new PID(0.21, 0.01, 0.000, 0.0001);
        pid_y = new PID(0.21, 0.01, 0.000, 0.0001);
        pid_theta = new PID(0.85, 0.1, 0.000, Math.toRadians(0.0001));

//        pid_x = new PID(0.045, 0.01, 0.000, 0.0001);
//        pid_y = new PID(0.045, 0.01, 0.000, 0.0001);
//        pid_theta = new PID(0.55, 0.1, 0.000, Math.toRadians(0.0001));

//        pid_x = new PID(0.04, 0.001, 0.000, 0.0001);
//        pid_y = new PID(0.04, 0.001, 0.000, 0.0001);
//        pid_theta = new PID(0.08, 0.002, 0.000, Math.toRadians(0.0001));

        waypoint = null;
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        motor_correction_enabled = true;
        initHardware();
    }

    public Drivetrain(HardwareMap hardwareMap, OpMode opMode, Pose pose) {
        this(hardwareMap, opMode);
        this.pose = pose;
    }

    public String getCurrentWaypoint() {
        if (waypoint == null) {
            return "NULL";
        } else {
            return String.format("%.2f, %.2f, %.2f", waypoint.x, waypoint.y, waypoint.theta);
        }
    }

    public void close_log() {
        log.close();
    }

    public void setPose (Pose pose) {
        this.pose = pose;
    }

    public Pose getPose() {
        return this.pose;
    }
    public void refreshPose() {
        // current encoder values (in counts or ticks).
        int currLeftPos = encoderLeft.getCurrentPosition();
        int currRightPos = encoderRight.getCurrentPosition();
        int currAuxPos = encoderAux.getCurrentPosition();

        // how much did each encoder count change?
        int dnLeft = currLeftPos - prevLeftPos;         // dn1
        int dnRight = currRightPos - prevRightPos;      // dn2
        int dnAux = currAuxPos - prevAuxPos;            // dn3

        // find change in center of robot in robot coordinate system.
        double dtheta = (dnRight - dnLeft) / POD_SPACING_LR_CM / POD_COUNTS_PER_CM;
        double dx = (dnRight + dnLeft) / 2.0 / POD_COUNTS_PER_CM;
        double dy = (dnAux - POD_SPACING_AUX_CM * (dnRight - dnLeft) / POD_SPACING_LR_CM) / POD_COUNTS_PER_CM;

        // find changes in field coordinates.
        double h = pose.theta + dtheta / 2.0;
        double deltaX = dx * Math.cos(h) - dy * Math.sin(h);
        double deltaY = dx * Math.sin(h) + dy * Math.cos(h);
        double deltaTheta = dtheta;

        // update pose.
        pose.x += deltaX;
        pose.y += deltaY;
        pose.theta += deltaTheta;

        // current values become previous ones.
        prevLeftPos = currLeftPos;
        prevRightPos = currRightPos;
        prevAuxPos = currAuxPos;
    }

    public void clearWaypoint() {
        waypoint = null;
        elapsed_time.reset();
        previous_time = elapsed_time.milliseconds();
        pid_x.clear();;
        pid_y.clear();
        pid_theta.clear();
    }

    public void setWaypoint(Pose waypoint) {
        this.waypoint = waypoint;
        elapsed_time.reset();
        previous_time = elapsed_time.milliseconds();
        pid_x.clear();;
        pid_y.clear();
        pid_theta.clear();
    }

    public boolean applyCorrection() {
        // do we have a valid waypoint?
        if (waypoint == null) {
            // return true because there is no waypoint to navigate to.
            return true;
        }

        // calculate error (in field coordinate system).
        double err_x = waypoint.x - pose.x;
        double err_y = waypoint.y - pose.y;
        double err_theta = waypoint.theta - pose.theta;

        // clip the error otherwise it can wash out the other error.
        err_x = Range.clip(err_x, -10, 10);
        err_y = Range.clip(err_y, -10, 10);

        // convert the x and y components from field coordinates to local (robot) coordinates.
        double err_x_local = Math.cos(pose.theta) * err_x + Math.sin(pose.theta) * err_y;
        double err_y_local = -Math.sin(pose.theta) * err_x + Math.cos(pose.theta) * err_y;

        // are we there yet?
        // use field coordinates.
        if (Math.abs(err_x) < 1 && Math.abs(err_y) < 1 && Math.abs(err_theta) < Math.toRadians(2)) {
            this.stop();
            this.clearWaypoint();
            pid_x.clear();
            pid_y.clear();
            pid_theta.clear();
            return true;
        }

        // figure out correction values for motors.
        double power_x_local;
        double power_y_local;
        double power_theta;

        // update current time.
        current_time = elapsed_time.milliseconds();
        delta_time = (current_time - previous_time) / 1000.0;
        previous_time = current_time;

        // calculate correction values using PID controllers (x, y, and theta).
        power_x_local = pid_x.calculate(err_x_local, delta_time);
        power_y_local = pid_y.calculate(err_y_local, delta_time);
        power_theta = pid_theta.calculate(err_theta, delta_time);

        // WHAT WOULD HAPPEN IF I OMIT THIS STEP?
        // clip the values so that the motor power stays within a reasonable range of values.
        // this also helps prevent one value from washing out the other two values too dramatically.
        power_x_local = Range.clip(power_x_local, -1, 1);
        power_y_local = Range.clip(power_y_local, -1, 1);
        power_theta = Range.clip(power_theta, -1, 1);

//        // apply power to each motor based on inverse kinematics for mecanum drive.
//        double powerFL = (power_x_local - power_y_local - (TRACK_WIDTH + WHEELBASE) / 2.0 * power_theta) / DRIVE_WHEEL_RADIUS_CM;
//        double powerBL = (power_x_local + power_y_local - (TRACK_WIDTH + WHEELBASE) / 2.0 * power_theta) / DRIVE_WHEEL_RADIUS_CM;
//        double powerBR = (power_x_local - power_y_local + (TRACK_WIDTH + WHEELBASE) / 2.0 * power_theta) / DRIVE_WHEEL_RADIUS_CM;
//        double powerFR = (power_x_local + power_y_local + (TRACK_WIDTH + WHEELBASE) / 2.0 * power_theta) / DRIVE_WHEEL_RADIUS_CM;

        // apply power to each motor.
        // use simplified inverse kinematic model.
        double powerFL = (power_x_local - power_y_local - power_theta);
        double powerBL = (power_x_local + power_y_local - power_theta);
        double powerBR = (power_x_local - power_y_local + power_theta);
        double powerFR = (power_x_local + power_y_local + power_theta);

        // don't normalize by the largest magnitude because the other corrections will get washed out by the largest one.
        // simply clip the power values so it will be in the accepted ranges.
        double max_power = 1.0;
        powerFL = Range.clip(powerFL, -max_power, +max_power);
        powerBL = Range.clip(powerBL, -max_power, +max_power);
        powerBR = Range.clip(powerBR, -max_power, +max_power);
        powerFR = Range.clip(powerFR, -max_power, +max_power);

        if (motor_correction_enabled) {
            // apply power.
            motorFL.setPower(powerFL);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);
            motorFR.setPower(powerFR);
        }

        // log data.
        log.log(String.format("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                pose.x, pose.y, pose.theta, err_x, err_y, err_theta, err_x_local, err_y_local,
                power_x_local, power_y_local, power_theta, powerFL, powerBL, powerBR, powerFR)
                );

        return false;
    }

    private void initHardware() {
        // Get and configure drive motors.
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

        // get and configure encoders.
        if (USE_ODOMETRY_POD) {
            encoderLeft = hardwareMap.dcMotor.get("motorBL");
            encoderRight = hardwareMap.dcMotor.get("intake");
            encoderAux = hardwareMap.dcMotor.get("motorBR");

            encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // set directions.
            encoderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            encoderRight.setDirection(DcMotorSimple.Direction.FORWARD);
            encoderAux.setDirection(DcMotorSimple.Direction.REVERSE);

            prevLeftPos = encoderLeft.getCurrentPosition();
            prevRightPos = encoderRight.getCurrentPosition();
            prevAuxPos = encoderAux.getCurrentPosition();
        } else {
            encoderLeft = motorFL;
            encoderAux = motorFL;
        }

        initIMU();
    }

    /**
     * calibrate encoder
     * reset the strafing and drive encoders
     */
    public void resetOdometry() {
        // reset encoders.
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set prev values to zero.
        prevLeftPos = 0;
        prevRightPos = 0;
        prevAuxPos = 0;

        // reset pose.
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
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
     * @param twist counterclockwise / clockwise (negative) power - follows right hand rule
     */
    public void drive(double drive,  double strafe, double twist) {
        // apply power to each motor based on inverse kinematics.
        double powerFL = (drive + strafe - twist);
        double powerBL = (drive - strafe - twist);
        double powerBR = (drive + strafe + twist);
        double powerFR = (drive - strafe + twist);

        // scale the powers (so they are <= 1).
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR), Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Only apply scalar if greater than 1. Otherwise we could unintentionally increase power
        // This also prevents dividing by 0
        if(scalar < 1) {
            scalar = 1;
        }

        powerFL /= scalar;
        powerBL /= scalar;
        powerFR /= scalar;
        powerBR /= scalar;

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