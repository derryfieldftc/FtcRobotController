package org.firstinspires.ftc.teamcode.Helper;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.common.util.report.qual.ReportOverride;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name="Camera")
public class Camera extends LinearOpMode {

    LinearOpMode opMode;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, String cameraName, LinearOpMode opMode) {
        initTfod(hardwareMap, cameraName);
        this.opMode = opMode;
    }

    @Override
    public void runOpMode(){}


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod(HardwareMap hardwareMap, String cameraName) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, cameraName));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);
    }

    public boolean detectPixel(int maximumCaptures, int captureTime, int minimumCaptureThreshold) {
        opMode.telemetry.addData("Active", opMode.opModeIsActive());
        opMode.telemetry.update();
        while (opMode.opModeIsActive()) {
            List<Recognition> totalRecognitions = tfod.getRecognitions();
            for (int i = 0; i < maximumCaptures; i++){
                for (Recognition recognition : tfod.getRecognitions()){
                    totalRecognitions.add(recognition);
                }
                opMode.telemetry.addData("Total Recognitions", totalRecognitions);
                opMode.telemetry.update();
                sleep(captureTime);
            }
            if (totalRecognitions.size() >= minimumCaptureThreshold)
                return true;
            return false;
        }
        return false;
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//
//    }   // end method telemetryTfod()

}   // end class
