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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.List;

@Autonomous(name = "CenterstageAutonomous")
public class CenterstageAutonomous extends AutoRobotStruct {

    private double x;
    private double y;
    private VisionDetectorOne visionDetector;
    private long startTime;
    private boolean positionSet = false;
    private IMU imu;  // Use BNO055IMU for BHI260AP
    private Orientation lastAngles = new Orientation();
    private startTime = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        visionDetector = new VisionDetectorOne(this);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Rest of your code...

        while (opModeIsActive() && !positionSet) {
            List<Recognition> recognitions = visionDetector.getRecognitions();
            visionDetector.telemetryTfod(this);
            telemetry.update();
            sleep(20);

            // Check if a model is detected within 5 seconds
            if (System.currentTimeMillis() - startTime < 5000 && !positionSet) {
                if (recognitions.size() > 0) {
                    Recognition recognition = recognitions.get(0);
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;

                    // Movement logic based on the detected position
                    if (x > -10 && x < 290) {
                        telemetry.addData("Left position", ".");
                        telemetry.addData("Line = ", atLine);
                        position = 1;
                        positionSet = true;
                    } else if (x > 300 && x < 575) {
                        telemetry.addData("Middle position", ".");
                        telemetry.addData("Line = ", atLine);
                        position = 2;
                        positionSet = true;
                    } else {
                        telemetry.addData("Right position", ".");
                        telemetry.addData("Line = ", atLine);
                        position = 3;
                        positionSet = true;
                    }
                }
            } else if (!positionSet) {
                // Assume position 3 after 5 seconds
                position = 3;
                positionSet = true;
            }

            // Move to the line if position is valid
            if (position > 0) {
                moveTowardLine();
            }
        }

        // Additional logic based on the detected position after reaching the line
        if (atLine && position > 0) {
            if (position == 1) {
                setDriverMotorPower(0.15, 0.15, 0.15, 0.15, 150);
                sleep(500);
                turnLeft(90); // Turn left 90 degrees using gyro
                moveTowardLine();
                dragBlock.setPosition(0.5);
                sleep(200);
            } else if (position == 2) {
                // Move forward for position 2
                setDriverMotorPower(0.15, 0.15, 0.15, 0.15, 150);
                sleep(500);
                dragBlock.setPosition(0.5);
                sleep(200);
                setDriverMotorPower(0.15, 0.15, 0.15, 0.15, 400);
                setDriverMotorPower(0.11, 0.11, 0.11, 0.11, 3000);
            } else if (position == 3) {
                // Turn right for position 3
                dragBlock.setPosition(0.5);
                // Use the gyro to turn right
                turnRight(90);
            }
        }
    }

    private void turnLeft(double targetAngle) {
        double initialAngle = getHeading();
        double currentAngle = initialAngle;

        while (opModeIsActive() && Math.abs(currentAngle - initialAngle) < targetAngle) {
            setDriverMotorPower(0.15, -0.15, 0.15, -0.15);
            telemetry.addData("Turning Left", "Current Angle: %.2f", currentAngle);
            telemetry.update();

            currentAngle = getHeading();
        }

        setDriverMotorZero();
    }

    private void turnRight(double targetAngle) {
        double initialAngle = getHeading();
        double currentAngle = initialAngle;

        while (opModeIsActive() && Math.abs(currentAngle - initialAngle) < targetAngle) {
            setDriverMotorPower(-0.15, 0.15, -0.15, 0.15);
            telemetry.addData("Turning Right", "Current Angle: %.2f", currentAngle);
            telemetry.update();

            currentAngle = getHeading();
        }

        setDriverMotorZero();
    }

    private double getHeading() {
    // Adjust the axes based on the actual orientation of the IMU on the robot
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return angles.firstAngle;
}


private void moveTowardLine() {
        while (!atLine && position > 0) {
            setDriverMotorPower(-0.11, -0.11, -0.11, -0.11);
            telemetry.addData("Moving to ", "LINE");
            telemetry.update();

            // Check for the line detection condition
            float[] hsvValues = readColor(colorSensor);
            if (hsvValues[0] < 25) {
                atLine = true;
                setDriverMotorZero();
                return; // Exit the loop after reaching the line
            } else {
                atLine = false;
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
