package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;

import java.util.List;

public class VisionDetectorOne extends AutoRobotStruct {

    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public VisionDetectorOne(LinearOpMode opMode) {
        initTfod(opMode);
    }

    public void initTfod(LinearOpMode opMode) {
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
        visionPortal.resumeStreaming();
    }

    public void stopDetection() {
        visionPortal.stopStreaming();
    }

    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }

    public List<Recognition> getObjects() {
        return tfod.getRecognitions();
    }

    public void telemetryTfod(LinearOpMode opMode) {
        List<Recognition> currentRecognitions = getRecognitions();
        opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            opMode.telemetry.addData("Object Position (X, Y)", "%.2f, %.2f", x, y);
        }

        opMode.telemetry.update();
    }
}
