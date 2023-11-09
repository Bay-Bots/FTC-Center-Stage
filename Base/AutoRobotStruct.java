package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


public class AutoRobotStruct extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    public Servo servoClaw1;
    public Servo servoClaw2;
    private DcMotor linearSlide;

    // accessed in RedDuck.java directly without get/set methods therefore not private vars
    public ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException { }

    public void initRunner() throws InternalError {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // 2
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // 3
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // 1
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // 0
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");



        // reverse direction for this drive train
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getDistanceFront() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
    //telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));



    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower, int s) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
        sleep(s);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(100);
    }  public void setClawPos(double pos1, double pos2) {
        servoClaw1.setPosition(pos1);
        servoClaw2.setPosition(pos2);
    }     public void translateRight(double m) {
        motorFrontRight.setPower(-m);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(-m);
        motorBackRight.setPower(m);
    }

    public void translateLeft(double m) {
        motorFrontRight.setPower(m);
        motorFrontLeft.setPower(-m);
        motorBackLeft.setPower(m);
        motorBackRight.setPower(-m);
        
    } 

}
