
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import android.graphics.Color;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CenterstageAutonomous")
public class CenterstageAutonomous extends AutoRobotStruct {
    private double x;
    private double y;
    private BNO055IMU imu;
    private VisionDetectorOne visionDetector;
    private long startTime;
    private boolean positionSet = false;

    private void initializeIMU() {
        try {
            // Initialize IMU
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.loggingEnabled = true;
            imuParameters.loggingTag = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            if (imu.isGyroCalibrated()) {
                telemetry.addData("IMU", "Calibration Complete");
            } else {
                telemetry.addData("IMU", "Calibration Failed");
            }
        } catch (Exception e) {
            telemetry.addData("IMU Initialization Error", e.getMessage());
            imu = null;
        }
        telemetry.update();
    }

    public void runOpMode() {
        initializeIMU();
        initRunner();
        visionDetector = new VisionDetectorOne(this);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // ... (existing code)

        waitForStart();

        startTime = System.currentTimeMillis();
        while (opModeIsActive() && !positionSet) {
            if (System.currentTimeMillis() - startTime < 5000) {
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
                        visionDetector.stopDetection();
                    } else if (x > 300 && x < 575) {
                        telemetry.addData("Middle position", ".");
                        telemetry.addData("Line = ", atLine);
                        position = 2;
                        visionDetector.stopDetection();
                    } else {
                        telemetry.addData("Right position", ".");
                        telemetry.addData("Line = ", atLine);
                        position = 3;
                        visionDetector.stopDetection();
                    }

                    // Additional logic based on the detected position after reaching the line
                    if (!atLine && position > 0) {
                        moveTowardLine();

                        if (position == 1) {
                            turnLeft();
                            dragBlock.setPosition(0.5);
                            setDriverMotorPower(0.15, 0.15, 0.15, 0.15, 100);
                        } else if (position == 2) {
                            setDriverMotorPower(-0.15, -0.15, -0.15, -0.15, 100);
                            dragBlock.setPosition(0.5);
                        } else if (position == 3) {
                            turnRight();
                            dragBlock.setPosition(0.5);
                            setDriverMotorPower(0.15, 0.15, 0.15, 0.15, 100);
                        }
                    }
                }
            }
        }
    }

    private void turnLeft() {
        setDriverMotorPower(0.20, -0.20, 0.20, -0.20, 1500);
        telemetry.addData("Turning Left", "");
        telemetry.update();
    }

    private void turnRight() {
        setDriverMotorPower(-0.20, 0.20, -0.20, 0.20, 1500);
        telemetry.addData("Turning Right", "");
        telemetry.update();
    }

    private void moveTowardLine() {
    long startTime = System.currentTimeMillis();

    // Initialize encoder targets
    int targetPosition = 2000; // Adjust this value based on your encoder counts

    while (!atLine && position > 0 && opModeIsActive()) {
        // Move using encoders
        motorFrontLeft.setTargetPosition(targetPosition);
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(0.5);

        motorFrontRight.setTargetPosition(targetPosition);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(0.5);

        motorBackLeft.setTargetPosition(targetPosition);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(0.5);

        motorFrontRight.setTargetPosition(targetPosition);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(0.5);

        while (motorFrontRight.isBusy()) {
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }

        // Rest of your code...
    }

    // Reset encoders after reaching the line
    setDriverMotorZero();

    telemetry.addData("Moving to ", "LINE");
    telemetry.addData("Elapsed Time", "%d ms", System.currentTimeMillis() - startTime);
    telemetry.update();

    // Check for the line detection condition
    float[] hsvValues = readColor(colorSensor);
    telemetry.addData("Hue Value", "%.2f", hsvValues[0]);

    if (hsvValues[0] < 25) {
        atLine = true;
        setDriverMotorZero();
        telemetry.addData("Line Detected", "Exiting the loop");
        return;
    } else {
        atLine = false;
    }

    sleep(100);

    if (System.currentTimeMillis() - startTime > 5000) {
        telemetry.addData("Timeout", "Line detection took too long.");
        positionSet = true;
        return;
    }
}

    private void moveToTouch() {
        while (!leftSensor.isPressed() || !rightSensor.isPressed()) {
            setDriverMotorPower(-0.11, -0.11, -0.11, -0.11);
            if (leftSensor.isPressed() || rightSensor.isPressed()) {
                setDriverMotorZero();
                break;  // Add this line to exit the loop
            }
        }
    }

    public float[] readColor(ColorSensor colorSensor) {
        float hsvValues[] = {0F, 0F, 0F};
        colorSensor.enableLed(true); // Assuming you want to turn ON the LED
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues;
    }
}
