package org.firstinspires.ftc.teamcode.teach;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeachBot {
    HardwareMap hardwareMap;

    public static final double PIVOT_SCALE = 0.75;
    public static final double DRIVE_SCALE = 0.75;

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    // constructor.
    public TeachBot(HardwareMap hardwareMap) {
        // we need hardware map so we can get references to the motors, sensors, etc..
        this.hardwareMap = hardwareMap;

        // get references to the motors.
        motorFL = this.hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = this.hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = this.hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = this.hardwareMap.get(DcMotor.class, "motorBR");

        // set motor directions.
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        // place motors into RUN_USING_ENCODER mode.
        // this mode is effectively a constant velocity mode.
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set motors so they brake when no power is applied.
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // drive the omnibot using primary inputs.
    // i really like the way the Wizards.EXE (Team #9794) explained how to program a mecanum/omni drive.
    // horizontal is movement in the side-to-side (x) direction.
    // vertical is movement in the front-and-back (y) direction.
    // pivot is rotation about the axis normal to the plane of the robot's travel (i.e., rotation about the z axis).
    // i assume the positive vertical (Y) direction points towards the front of the robot
    //
    //                          ^  Y (vertical)
    //                          |
    //                          |
    //                          +----->        X (horizontal)
    public void drive(double horizontal, double vertical, double pivot)  {
        // scale the values before using them.
        pivot *= PIVOT_SCALE;
        horizontal *= DRIVE_SCALE;
        vertical *= DRIVE_SCALE;

        // Set motor powers
        double powerFL = vertical + horizontal - pivot;
        double powerFR = vertical - horizontal + pivot;
        double powerBL = vertical - horizontal - pivot;
        double powerBR = vertical + horizontal + pivot;

        // scale the powers (so they are <= 1).
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Only apply scalar if greater than 1. Otherwise, we could unintentionally increase power
        // This also prevents dividing by 0
        if(scalar < 1)
            scalar = 1;

        // Apply scalar
        powerFL /= scalar;
        powerFR /= scalar;
        powerBL /= scalar;
        powerBR /= scalar;

        // drive the motors.
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    public void stop() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
}
