package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;

@Autonomous(name="AutoBot22_23ReverseRight")
public class AutoBot22_23ReverseRight extends AutoRobotStruct {
    private ColorSensor colorSensor;
    private boolean bLedOn = true;
    private String estColor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        
        colorSensor.enableLed(bLedOn);
        initRunner();
        telemetry.addData("STATUS", "Initialized");
        telemetry.update();
        servoClaw1.setPosition(0.4);
        servoClaw2.setPosition(1);
        waitForStart();
        while(opModeIsActive()) {
            
            telemetry.addData("%s", estColor);
            telemetry.addData("STATUS", "Ready!");
            telemetry.update();
            while (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 9) {
            telemetry.addData("cm", "%.2f cm", (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)));
            telemetry.update();
            setDriverMotorPower(0.25,0.25,0.25,0.25);
            }
            setDriverMotorPower(-0,-0,-0,-0);
            sleep(750);
            float[] colorValues = readColor(colorSensor);
                
            if (colorValues[1] < 0.370) {
                estColor = ("Red");
                telemetry.addData("Red", colorValues[1]);
                telemetry.update();
                setDriverMotorPower(0.4,0.4,0.4,0.4);
                sleep(1000);
                setDriverMotorPower(0.5,-0.5,0.5,-0.5, 1550);
                sleep(250);
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(1800);
            } else if (colorValues[1] > 0.450 && colorValues[0] > 130) {
                estColor = ("Green");
                telemetry.addData("Green", colorValues[1]);
                telemetry.update();
                setDriverMotorPower(0.25,0.25,0.25,0.25, 2000);
            } else {
                estColor = ("Black");
                telemetry.addData("Black", colorSensor.blue());
                telemetry.update();
                setDriverMotorPower(0.4,0.4,0.4,0.4);
                sleep(1000);
                setDriverMotorPower(-0.5,0.5,-0.5,0.5, 1500);
                sleep(250);
                setDriverMotorPower(0.5 ,0.5,0.5,0.5);
                sleep(2250);

            }
            break;
        }
    }
    
    public float[] readColor(ColorSensor colorSensor) {
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = false;

        boolean bLedOn = true;
        
        bPrevState = bCurrState;
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", "%.3f", hsvValues[1]);
        telemetry.update();
        
        // int[] colorValues = {colorSensor.red(), colorSensor.green(), colorSensor.blue()};
        return hsvValues;
    }
}