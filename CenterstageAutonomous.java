package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.VisionDetectorOne;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.List;


@Autonomous(name = "CenterstageAutonomous")
public class CenterstageAutonomous extends AutoRobotStruct {

    // Declare global variables
    private double x;
    private double y;

    // Create an instance of VisionDetector
    VisionDetectorOne visionDetector;

    // Create color sensor object
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        visionDetector = new VisionDetectorOne(this);

        // Declare motors and servos
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Start TensorFlow Object Detection
        visionDetector.startDetection();

        waitForStart();

        while (opModeIsActive()) {


            // Read color values
            float[] hsvValues = readColor(colorSensor);
            List<Recognition> recognitions = visionDetector.getRecognitions();
            visionDetector.telemetryTfod(this);
            telemetry.update();
            sleep(20);

            if (recognitions.size() > 0) {
                Recognition recognition = recognitions.get(0);
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;

            } else {
                
            }
            // Check if the robot is on the line

            // Update telemetry data
            telemetry.addData("At Line: ", atLine);
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.update();
        }
    }

    public float[] readColor(ColorSensor colorSensor) {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = false;

        boolean bLedOn = true;

        bPrevState = bCurrState;
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        return hsvValues;
    }
}
