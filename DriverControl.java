package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.base.RobotStructure;

@TeleOp(name="DriverControl")
public class DriverControl extends RobotStructure {
    

    public void loop() {

    initDriver();

    while (gamepad1.dpad_down) {
    setDriverMotorPower(0.25,0.25,0.25, 0.25);

        if (!gamepad1.dpad_down){
        setDriverPowerZERO();
    }
}

while (gamepad1.dpad_up){
    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);

     if (!gamepad1.dpad_up){
        setDriverPowerZERO();
    }
}

while (gamepad1.dpad_right){
    translateRight(0.25);

     if (!gamepad1.dpad_right){
        translateRight(0);
    } 
}
while (gamepad1.dpad_left){
    translateLeft(0.25);

      if (!gamepad1.dpad_left){
        translateLeft(0);
    }
}  

if (gamepad2.dpad_down) {
    Arm2.setPower(0.1);
}   else if (gamepad2.dpad_up) {
    Arm2.setPower(-0.1);
}    
    
//if(gamepad2.a) {
//setClawPos(1, 1);opens 
//servoClaw1.setPosition(0.6);
//servoClaw2.setPosition(0.6);
//}

//if(gamepad2.b) {
// setClawPos(0.5, 0.5); closes
//servoClaw1.setPosition(.1);
//servoClaw2.setPosition(.9);
//}

if(gamepad2.a) {
//setClawPos(1, 1);opens 
//servoClaw1.setPosition(0.55);
servoClaw2.setPosition(0.59);
}

if(gamepad2.b) {
// setClawPos(0.5, 0.5); closes
//servoClaw1.setPosition(0.35);
servoClaw2.setPosition(.34);
}

    
if (gamepad1.a) {
        tapeMotor.setPower(1.0);
    } else if (gamepad1.b) {
        tapeMotor.setPower(-1.0);
    } else {
        tapeMotor.setPower(0);
    }
}
}