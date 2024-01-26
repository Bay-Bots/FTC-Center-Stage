
package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Use DcMotorEx for encoder support
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class AutoRobotStruct extends LinearOpMode {
    public GyroSensor gyroSensor;
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor Arm1;
    public DcMotor Arm2;
    public DcMotor chainMotor;
    public Servo servoClaw1;
    public Servo servoClaw2;
    public Servo dragBlock;
    public Servo launcher;
    public Servo tapeHold;
    public Servo tapeRelease;
    public ColorSensor colorSensor; // CH I2C 2
    public boolean bLedOn = true;
    public boolean atLine = false;
    public String estColor;
    public int position = 0;
    public TouchSensor leftSensor;
    public TouchSensor rightSensor;

    @Override
    public void runOpMode() throws InterruptedException {}

    public void initRunner() throws InternalError {
         gyroSensor = hardwareMap.get(GyroSensor.class, "gyroSensor"); // EH I2C 2
        servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1"); // EH 5
        servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2"); // EH 2
        tapeRelease = hardwareMap.get(Servo.class, "tapeRelease"); // EH 3
        tapeHold = hardwareMap.get(Servo.class, "tapeHold"); // EH 4
        dragBlock = hardwareMap.get(Servo.class, "dragBlock"); // EH 1 
        launcher = hardwareMap.get(Servo.class, "launcher"); // EH 0
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // 3
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // 2
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // 1
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // 0
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam1"); // Adjust name if needed
        
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1"); 
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2"); 
        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor"); 
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        dragBlock.setPosition(0.05);
        // launcher.setPosition(.9);
        // servoClaw2.setPosition(.705);   
        // servoClaw1.setPosition(.375);
        // tapeRelease.setPosition(0.43);
        // tapeHold.setPosition(0.7);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

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
    }

    public void setDriverMotorZero() {
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(100);
    }

    public void translateRight(double m) {
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
    } public void driveForward (int targetPosition) {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFrontRight.setTargetPosition(targetPosition);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(.15);
        
        motorFrontLeft.setTargetPosition(targetPosition);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(.15);
        
        motorBackRight.setTargetPosition(targetPosition);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(.15);
        
        motorBackLeft.setTargetPosition(targetPosition);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(.15);
        while (motorFrontRight.isBusy() ){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    } public void rotateLeft() {
      int targetPosition = 1140;
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFrontRight.setTargetPosition(-targetPosition);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(.15);
        
        motorFrontLeft.setTargetPosition(targetPosition);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(.15);
        
        motorBackRight.setTargetPosition(-targetPosition);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(.15);
        

        motorBackLeft.setTargetPosition(targetPosition);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(.15);
        
          while (motorFrontRight.isBusy() ){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }
        
    }public void rotateRight() {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = 1140;
        motorFrontRight.setTargetPosition(targetPosition);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(.15);
        
        motorFrontLeft.setTargetPosition(-targetPosition);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(.15);
        
        motorBackRight.setTargetPosition(targetPosition);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(.15);
        
        motorBackLeft.setTargetPosition(-targetPosition);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(.15);
        
          while (motorFrontRight.isBusy() ){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }
        
    } public void driveBackward(int targetPosition) {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFrontRight.setTargetPosition(-targetPosition);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(.25);
        
        motorFrontLeft.setTargetPosition(-targetPosition);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(.25);
        
        motorBackRight.setTargetPosition(-targetPosition);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(.25);
        
        motorBackLeft.setTargetPosition(-targetPosition);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(.25);
        
          while (motorFrontRight.isBusy() ){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }
        
         motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
    
    public void scorePixel() {
        armMove(100);      // Slight move on arm
        chainMove(138);      // Raise chain motor
        armMove(932);   // raise arm
        chainMove(1816);   // chain into scoring position
        armMove(720);   // arm into scoring position
        armMove(1078);   // arm out of way
        chainMove(138);   // chain out of way
        armMove(0);   // arm to zero
        chainMove(0);   // chain to zero
        sleep(5000);
    }
    public void armMove(int armPosition) {
            Arm1.setTargetPosition(armPosition);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPosition);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
         
        long  timeout = 2000;
        long  startTime = System.currentTimeMillis();
        while (Arm1.isBusy()){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
            if ((System.currentTimeMillis() - startTime) > timeout){
                break;}
        }  
            
        int encoderCount1 = Arm1.getCurrentPosition();
        telemetry.addData("Arm1 Value =", encoderCount1);
        
        int encoderCount2 = Arm2.getCurrentPosition();
        telemetry.addData("Arm2 Value =", encoderCount2);
        
        int encoderCountC = chainMotor.getCurrentPosition();
        telemetry.addData("ChainMotor Value =", encoderCountC);
       telemetry.update();
             sleep(6000);
    }
    
    public void chainMove(int chainPosition) {
            
            chainMotor.setTargetPosition(chainPosition);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.8);
        long  timeout = 20000;
        long  startTime = System.currentTimeMillis();
        while (chainMotor.isBusy()){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
            if ((System.currentTimeMillis() - startTime) > timeout){
                break;}
        }  
            
        int encoderCount1 = Arm1.getCurrentPosition();
        telemetry.addData("Arm1 Value =", encoderCount1);
        
        int encoderCount2 = Arm2.getCurrentPosition();
        telemetry.addData("Arm2 Value =", encoderCount2);
        
        int encoderCountC = chainMotor.getCurrentPosition();
        telemetry.addData("ChainMotor Value =", encoderCountC);
       telemetry.update();
             sleep(6000);
    }
}
