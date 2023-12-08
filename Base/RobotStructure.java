package org.firstinspires.ftc.teamcode.base;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static java.lang.Double.parseDouble;

public class RobotStructure extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    public Servo servoClaw1;
   public Servo servoClaw2;
    public DcMotor Arm1;
   public DcMotor Arm2;
    public TouchSensor limit1;
    public TouchSensor limit2;
    public DcMotor tapeMotor;
    public Servo dragBlock;
    public Servo launcher;
    public Servo tapeHold;
    public DcMotor chainMotor;
    public Servo tapeRelease;
    

    @Override
    public void init() {
        
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // 3
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // 2
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // 1
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // 0
        servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2");
        tapeRelease = hardwareMap.get(Servo.class, "tapeRelease");
        tapeHold = hardwareMap.get(Servo.class, "tapeHold");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1"); // 0
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2"); // 1http://192.168.43.1:8080/?page=java/editor.html?/src/org/firstinspires/ftc/teamcode/Base/RobotStructure.java&pop=true
        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor"); 
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tapeMotor = hardwareMap.get(DcMotor.class, "tapeMotor");
        dragBlock = hardwareMap.get(Servo.class, "dragBlock");
        launcher = hardwareMap.get(Servo.class, "launcher");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    


    }

    @Override
    public void loop() {}
    public void initDriver(){
        float gamepad1LeftY = gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad2RightY = gamepad2.right_stick_y;
        float gamepad1RightX = -gamepad1.right_stick_x;
        float gamepad2LeftY = gamepad2.left_stick_y;
        float frontRightPower = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
        float frontLeftPower = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float backLeftPower = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float backRightPower = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
        float armPower = gamepad2RightY;
        float chainPower = gamepad2LeftY;
        //  float armPower2= gamepad2LeftY;


        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        Arm1.setPower(armPower);
        Arm2.setPower(armPower);  
        chainMotor.setPower(chainPower);
       
        int encoderCount = chainMotor.getCurrentPosition();
        telemetry.addData("encoder:", encoderCount);
        telemetry.update();
        
        int armPositionScore = 1400;
        int armPositionDrive = 419;
        int armPositionGrab = 297;
        int armPositionVertical = 1147;
        int chainPositionScore = 905;
        int chainPositionDrive = 1618;
        int chainPositionGrab = 1970;
        int chainPositionVertical = 1372;
        
          if (gamepad2.b) {
            Arm1.setTargetPosition(armPositionScore);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPositionScore);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
            chainMotor.setTargetPosition(chainPositionScore);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.6);
        
        long timeout = 2000;
        long startTime = System.currentTimeMillis();
        while (Arm1.isBusy() || chainMotor.isBusy() && (System.currentTimeMillis() - startTime < timeout)){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }}

        
         if (gamepad2.a) {
            Arm1.setTargetPosition(armPositionGrab);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPositionGrab);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
            chainMotor.setTargetPosition(chainPositionGrab);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.6);
        
        long timeout = 2000;
        long startTime = System.currentTimeMillis();
        while (Arm1.isBusy() || chainMotor.isBusy() && (System.currentTimeMillis() - startTime < timeout)){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }}
        
        if (gamepad2.x) {
            Arm1.setTargetPosition(armPositionDrive);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPositionDrive);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
            chainMotor.setTargetPosition(chainPositionDrive);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.6);
        
      long timeout = 2000;
        long startTime = System.currentTimeMillis();
        while (Arm1.isBusy() || chainMotor.isBusy() && (System.currentTimeMillis() - startTime < timeout)){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }}
        
         if (gamepad2.y) {
            Arm1.setTargetPosition(armPositionVertical);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPositionVertical);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
                        chainMotor.setTargetPosition(chainPositionVertical);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.6);
        
        long timeout = 2000;
        long startTime = System.currentTimeMillis();
        while (Arm1.isBusy() || chainMotor.isBusy() && (System.currentTimeMillis() - startTime < timeout)){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
        }}

        //  Launch Plane
   if (gamepad1.right_bumper) {
        launcher.setPosition(.945);
    }
    
// Plane initialization
    if (gamepad1.left_bumper) {
        launcher.setPosition(.9);
    }
    
    // Release tape measure
    if (gamepad2.right_bumper) {
       // tapeRelease.setPosition(.39);
        tapeHold.setPosition(.50);
    }
    
     if (gamepad1.y) {
       // tapeRelease.setPosition(.39);
        tapeHold.setPosition(.70);
    }
    
// Return tape servo, sweeper servos to home position
    if (gamepad2.left_bumper) {
        tapeRelease.setPosition(.43);
    }
    
    
if(gamepad1.x) {    //  Open Main
//setClawPos(1, 1);opens 
//servoClaw1.setPosition(0.55);
servoClaw2.setPosition(0.485);
}

if(gamepad1.b) {     // Open Both
servoClaw1.setPosition(0.62);    
servoClaw2.setPosition(0.485);
}

if(gamepad1.a) {    //  Close both pixel holders
// setClawPos(0.5, 0.5); closes
//servoClaw1.setPosition(0.35);
servoClaw2.setPosition(.705);    // 2nd Claw
servoClaw1.setPosition(.375);    // Main claw
}


    
    
        if (gamepad2.dpad_down)  {
            tapeMotor.setPower(-1.0);
        }
        else if (gamepad2.dpad_up)  {
            tapeMotor.setPower(1.0);
        }
        else  {
            tapeMotor.setPower(0);
        }
         if (gamepad1.y)  {
          // tapeMotor.setPower(-0.3);
        }
        int duration = 2000; // Duration in milliseconds

//





        /*
        Plug in arm motor to slot 0 on expansion hub (Not control hub)
        Go into driver tablet, go to configuration, obhs, control hub portal -> expansion hub-> DC motors
        Type "Arm1" into slot 0
        press done and save.
        If we end up using 2 motors then add Arm2 in slot 1.
        will move when you move right joystick up and down on gamepad #2
        */
            
    }  public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
    }

    public void setDriverPowerZERO() {
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
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
    
    } public void quickStop() {
        tapeMotor.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    
    
}
