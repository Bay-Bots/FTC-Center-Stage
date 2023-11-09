package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionDetector {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/blue1.tflite";
    private static final String[] LABELS = {"Team Element"};

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public VisionDetector(LinearOpMode opMode) {
        initTfod(opMode);
    }

    private void initTfod(LinearOpMode opMode) {
        tfod = new TfodProcessor.Builder()
                .setModelFileName("blue1.tflite")
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    public void startDetection() {
        visionPortal.stopStreaming();
    }

    public void stopDetection() {
        visionPortal.resumeStreaming();
    }

    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }

    public void telemetryTfod(LinearOpMode opMode) {
        List<Recognition> currentRecognitions = getRecognitions();
        opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            opMode.telemetry.addData("", " ");
            opMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            opMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            opMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
