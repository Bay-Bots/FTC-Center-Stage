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
    DcMotor motorFrontRight; // Control hub 1 = 2
    DcMotor motorFrontLeft;  // Control hub 1 = 1
    DcMotor motorBackRight;  // Control hub 1 = 0
    DcMotor motorBackLeft;   // Control hub 1 = 3
    public Servo servoClaw1;
   public Servo servoClaw2;
    public DcMotor Arm1;        // Expansion 0
   public DcMotor Arm2;         //  Expansion 3
    public TouchSensor limit1;
    public TouchSensor limit2;
    public DcMotor tapeMotor;   //  Expansion 2
    public Servo dragBlock;
    public Servo launcher;
    public Servo tapeHold;
    public DcMotor chainMotor;   //  Expansion  1
    public Servo tapeRelease;
    public boolean down; 
    

    @Override
    public void init() {
        down = false;
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); 
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); 
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); 
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); 
        servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2");
        tapeRelease = hardwareMap.get(Servo.class, "tapeRelease");
        tapeHold = hardwareMap.get(Servo.class, "tapeHold");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1"); 
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2"); 
        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor"); 
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
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
        
         dragBlock.setPosition(0.05);
        launcher.setPosition(.9);
       // servoClaw2.setPosition(.705);   
       // servoClaw1.setPosition(.375);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
   public void loop() {}
public void initDriver() {
    float gamepad1LeftY = gamepad1.left_stick_y;
    float gamepad1LeftX = gamepad1.left_stick_x;
    float gamepad2RightY = gamepad2.right_stick_y;
    float gamepad1RightX = -gamepad1.right_stick_x;
    float gamepad2LeftY = gamepad2.left_stick_y;

    // Adjust the signs for translation
    float translation = -gamepad1LeftX;
    float rotation = gamepad1RightX;

    float frontRightPower = -gamepad1LeftY + translation + rotation;
    float frontLeftPower = -gamepad1LeftY - translation - rotation;
    float backLeftPower = -gamepad1LeftY + translation - rotation;
    float backRightPower = -gamepad1LeftY - translation + rotation;
    float armPower = gamepad2RightY;
    float chainPower = gamepad2LeftY / 2;

    motorFrontLeft.setPower(-frontLeftPower);
    motorBackLeft.setPower(-backLeftPower);
    motorFrontRight.setPower(-frontRightPower);
    motorBackRight.setPower(-backRightPower);
    Arm1.setPower(armPower * 0.3);
    Arm2.setPower(armPower * 0.3);
    chainMotor.setPower(chainPower);

       int encoderCount1 = Arm1.getCurrentPosition();
        telemetry.addData("Arm1 Value =", encoderCount1);
        
        int encoderCount2 = Arm2.getCurrentPosition();
        telemetry.addData("Arm2 Value =", encoderCount2);
        
        int encoderCountC = chainMotor.getCurrentPosition();
        telemetry.addData("ChainMotor Value =", encoderCountC);
       telemetry.update();
        
        int armPositionScore = 1582;
        int armPositionDrive = 508;
        int armPositionGrab = 340;
        int armPositionVertical = 1180;
        
        int chainPositionScore = 944;
        int chainPositionDrive = 2164;
        int chainPositionGrab = 2098;
        int chainPositionVertical = 1592;
        
          if (gamepad2.b) {
              armEncoder(armPositionScore,chainPositionScore);
          }
        
           if (gamepad2.a) {
                armEncoder(armPositionGrab,chainPositionGrab);
           }
        
            if (gamepad2.x) {
                armEncoder(armPositionDrive,chainPositionDrive);
            }
        
             if (gamepad2.y) {
                armEncoder(armPositionVertical,chainPositionVertical);
             }

        //  Launch Plane
   if (gamepad1.right_bumper) {
        launcher.setPosition(.95);
    }
    
// Plane initialization
    if (gamepad1.left_bumper) {
        launcher.setPosition(.9);
    }
    
    // Release tape measure
    if (gamepad2.right_bumper) {
       tapeRelease.setPosition(.39);
     }
     
     if (gamepad1.left_bumper) {
      // tapeHold.setPosition(.45);
      motorFrontRight.setPower(.5);
     }
       
    if (gamepad1.y) {
       // tapeRelease.setPosition(.39);
      // PUT BACK IN tapeHold.setPosition(.70);
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
// wills attempt
int tinfoil = 0;
int elephant = 10;
if (gamepad2.dpad_down){
    while (tinfoil < 700000) {
    tapeMotor.setPower(-1);
    tinfoil = tinfoil + 1;
}
 while (elephant == 10) {
    tapeMotor.setPower(-.5);  
 }
 
}

   // 
    
        //if (gamepad2.dpad_down)  {
        //    tapeMotor.setPower(-1.0);
        //}
        else if (gamepad2.dpad_up)  {
            tapeMotor.setPower(1.0);
        }
        else  {
            tapeMotor.setPower(0);
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
    
    public void armEncoder(int armPosition, int chainPosition)  {
            Arm1.setTargetPosition(armPosition);
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setPower(.5);
            Arm2.setTargetPosition(armPosition);
            Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm2.setPower(.5);
            chainMotor.setTargetPosition(chainPosition);
            chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chainMotor.setPower(.6);
                 
      long  timeout = 2000;
      long  startTime = System.currentTimeMillis();
        while (Arm1.isBusy() || chainMotor.isBusy()){
            telemetry.addData("Moving to Position", "...");
            telemetry.update();
            if ((System.currentTimeMillis() - startTime) > timeout){
                break;}
        }  
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
}
