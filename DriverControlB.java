package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.base.StructB;

@TeleOp(name="DriverControlB")
public class DriverControlB extends StructB {
        

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
}
}
    