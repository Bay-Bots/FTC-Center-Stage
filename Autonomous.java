  package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "ConePositionDetectionWebcam")
public class ConePositionDetectionWebcam extends LinearOpMode {
    OpenCvCamera webcam;
    ConePositionPipeline conePipeline;
    Servo servoExample; // Adjust this line based on your servo configuration

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(com.qualcomm.robotcore.hardware.Camera.class, "webcamName"),
            cameraMonitorViewId
        );
        conePipeline = new ConePositionPipeline();
        webcam.setPipeline(conePipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });

        servoExample = hardwareMap.servo.get("servoExample"); // Adjust this line based on your servo configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int conePosition = conePipeline.getConePosition();

            // Implement your logic based on conePosition
            // For example, move the robot or manipulate components accordingly

            telemetry.addData("Cone Position", conePosition);
            telemetry.update();

            sleep(100);
        }
    }

    public static class ConePositionPipeline extends OpenCvPipeline {
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point REGION_TOP_LEFT = new Point(100, 50);
        static final int REGION_WIDTH = 120;
        static final int REGION_HEIGHT = 120;

        Mat regionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int conePosition = -1; // -1 indicates undetermined position

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            regionCb = Cb.submat(new Rect(REGION_TOP_LEFT, new Point(
                REGION_TOP_LEFT.x + REGION_WIDTH, REGION_TOP_LEFT.y + REGION_HEIGHT
            )));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            Imgproc.rectangle(
                input, REGION_TOP_LEFT, new Point(
                    REGION_TOP_LEFT.x + REGION_WIDTH,
                    REGION_TOP_LEFT.y + REGION_HEIGHT
                ), GREEN, 2
            );

            // Implement ftc-ml cone position classification here
            // Assign the determined cone position to the 'conePosition' variable

            return input;
        }

        public int getConePosition() {
            return conePosition;
        }
    }
}
