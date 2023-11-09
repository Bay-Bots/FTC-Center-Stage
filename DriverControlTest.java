//  package org.firstinspires.ftc.teamcode;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.teamcode.base.RobotStructure;
// 
// @TeleOp(name="DriverControlTest")
// public class DriverControl extends RobotStructure {
//     
// 
//     public void loop() {
// 
//     initDriver();
// 
//     while (gamepad1.dpad_down) {
//     setDriverMotorPower(0.25,0.25,0.25, 0.25);
// 
//         if (!gamepad1.dpad_down){
//         setDriverPowerZERO();
//     }
// }
// 
// while (gamepad1.dpad_up){
//     setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
// 
//      if (!gamepad1.dpad_up){
//         setDriverPowerZERO();
//     }
// }
// 
// while (gamepad1.dpad_right){
//     translateRight(0.25);
// 
//      if (!gamepad1.dpad_right){
//         translateRight(0);
//     } 
// }
// while (gamepad1.dpad_left){
//     translateLeft(0.25);
// 
//       if (!gamepad1.dpad_left){
//         translateLeft(0);
//     } 
// }  
// while (gamepad2.dpad_up){
//     linearSlide.setPower(-0.5);
// 
//      if (!gamepad2.dpad_up){
//     linearSlide.setPower(0);
//     }
// }
// while (gamepad2.dpad_down){
//     linearSlide.setPower(0.5);
//      if (!gamepad2.dpad_down){
//     linearSlide.setPower(0);
//     }
// }
// 
// //        
//         if(gamepad2.a) {
//             // setClawPos(1, 1);opens 
//             servoClaw1.setPosition(0.8);
//             servoClaw2.setPosition(.85);
//         }
// 
// //        
//         if(gamepad2.b) {
//        // setClawPos(0.5, 0.5); closes
//         servoClaw1.setPosition(1);
//         servoClaw2.setPosition(0.5);
// 
// 
// //            servoClaw1.setPosition(0.8);
// //            servoClaw2.setPosition(.85);
// }   if(gamepad1.a) {
//     servo65.setPosition(1);
// } else if (gamepad1.b){
//     servo65.setPosition(0);
// }
// }
// }
// 