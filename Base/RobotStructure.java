package org.firstinspires.ftc.teamcode.base;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    DcMotorEx motorArmLeft;
    DcMotorEx motorArmRight;
   public Servo servoClaw1;
   public Servo servoClaw2;
    public DcMotor Arm1;
   public DcMotor Arm2;
    public TouchSensor limit1;
    public TouchSensor limit2;
    public Servo servo65;
    public DcMotor tapeMotor;
    public Servo 

    @Override
    public void init() {
        
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // 3
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // 2
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // 1
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // 0
        servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1"); // 1
        servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2"); // 2
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1"); // 0
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2"); // 1
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo65 = hardwareMap.get(Servo.class, "servo65");
        tapeMotor = hardwareMap.get(DcMotor.class, "tapeMotor");
        d

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
        //  float armPower2= gamepad2LeftY;


        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        Arm1.setPower(armPower);
        Arm2.setPower(armPower);    
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
    
    }  public void setClawPos(double pos1, double pos2) {
        servoClaw1.setPosition(pos1);
        servoClaw2.setPosition(pos2);
        
    } public void quickStop() {
        tapeMotor.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    
    
}