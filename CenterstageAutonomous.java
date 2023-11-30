package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import android.graphics.Color;
import java.util.List;

@Autonomous(name = "CenterstageAutonomous")
public class CenterstageAutonomous extends AutoRobotStruct {

    private double x;
    private double y;
    private VisionDetectorOne visionDetector;


    @Override
    public void runOpMode() {
        visionDetector = new VisionDetectorOne(this);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        visionDetector.startDetection();
        initRunner();

        waitForStart();

        while (opModeIsActive()) {
            float[] hsvValues = readColor(colorSensor);
            List<Recognition> recognitions = visionDetector.getRecognitions();
            visionDetector.telemetryTfod(this);
            telemetry.update();
            sleep(20);

            if (recognitions.size() > 0) {
                Recognition recognition = recognitions.get(0);
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;

                // Movement logic based on the detected position
                if (x > -10 && x < 290) {
                    telemetry.addData("Left position", ".");
                    telemetry.addData("Line = ", atLine);
                    position = 1;
                    sleep(250);
                    visionDetector.stopDetection();

                } else if (x > 300 && x < 575) {
                    telemetry.addData("Middle position", ".");
                    telemetry.addData("Line = ", atLine);
                    position = 2;
                    sleep(250);
                    visionDetector.stopDetection();


                } else {
                    telemetry.addData("Right position", ".");
                    telemetry.addData("Line = ", atLine);
                    position = 3;
                    sleep(250);
                    visionDetector.stopDetection();

                }
            }
                // Move to the line (you can adjust the power and duration)
                while (!atLine && position > 0) {
                    setDriverMotorPower(-0.15,-0.15,-0.15,-0.15);
                    telemetry.addData("ON", "LINE");
                if (atLine) {
                    setDriverMotorZero();
                telemetry.addData("ON", "LINE");
                atLine = false; // Reset the atLine variable
    }
}

            // Update telemetry data
            telemetry.addData("At Line: ", atLine);
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.update();
        }
    }

    public float[] readColor(ColorSensor colorSensor) {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues;
    }
}
