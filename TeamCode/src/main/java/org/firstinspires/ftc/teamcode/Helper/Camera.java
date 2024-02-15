package org.firstinspires.ftc.teamcode.Helper;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class Camera {

    private boolean USE_WEBCAM;
    private String CAMERA_NAME;
    private String TFOD_MODEL_FILE_OR_ASSET;
    private String[] LABELS = {};

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public Camera(boolean useWebcam,
                  String cameraName,
                  String tfodModelFile,
                  String[] labels,
                  HardwareMap hardwareMap) {
        USE_WEBCAM = useWebcam;
        CAMERA_NAME = cameraName;
        TFOD_MODEL_FILE_OR_ASSET = tfodModelFile;
        LABELS = labels;
        initTfod(hardwareMap);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE_OR_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);
    }

    public boolean recognizeByBoolean(String label) {
//        boolean isLabelValid = false;
//        List<String> labels = recognizeByLabel();
//        for (String _label : LABELS) {
//            if (label == _label) {
//                isLabelValid = true;
//            }
//        }
        List<String> labels = recognizeByLabel();
        for (String _label : labels) {
            if (_label == label) {
                return true;
            }
        }
        return false;
    }

    public List<String> recognizeByLabel() {
        List<String> labels = new ArrayList<String>();
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        for (Recognition recognition : currentRecognitions) {
            labels.add(recognition.getLabel());
        }
        return labels;
    }

    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }
}
