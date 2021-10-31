package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="FFblueLeft", group="HGT")

public class FFblueLeft extends AutoBot{
    @Override
    public void runOpMode() {
        waitForStart();
        setup();

        ObjectDetector ob = new ObjectDetector(hardwareMap);
        ob.init();
        String label;

        armup(robot);

        spin(robot, 20, 3);

        move(robot, 0.1, 10, 6, LEFT);
        sleep(130);

        move(robot, 0.75, 10, 6, FORWARD);
        sleep(150);
    }
}
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Autonomous(name="AutonomousMode", group="HGT")
//public class AutonomousModeBlue extends AutoBot {
//
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    @Override
//
//    public void runOpMode() {
//        setup();
//
//        ObjectDetector ob = new ObjectDetector(hardwareMap);
//        ob.init();
//        String label;
//
//
//        waitForStart();
//
//
//        label = ob.detectRingLabel();
//        telemetry.addData("# Object Detected:  ", label);
//        telemetry.update();
//
//
//        closeClaw(robot);
//        //sleep(150);
//        //move(robot, 0.45, 6, 6, LEFT);
//        if(label == null) {
//            squareA();
//        } else if(label.equals("Single")) {
//            squareB();
//        } else if(label.equals("Quad")) {
//            squareC();
//        }
//
//
//
//    }
//    public void squareA() {
//        move(robot, 0.45, 6, 6, LEFT);//test
//        sleep(100); //test
//
//        move(robot, 0.75, 23, 6, FORWARD);
//        sleep(130);
//
//        armdown(robot);
//        sleep(800);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//
//        move(robot, 0.325, 16, 4, TURNRIGHT);
//
//        armdown(robot);
//
//        move(robot, 0.75, 16, 6, FORWARD);
//        sleep(150);
//
//        move(robot, 0.325, 3.4, 4, TURNRIGHT);
//        sleep(150);
//
//        closeClaw(robot);
//        sleep(1000);
//
//        armup(robot);
//
//        move(robot, 0.325, 21, 4, TURNLEFT);
//        sleep(150);
//
//        move(robot, 0.75, 17, 6, FORWARD);
//        sleep(150);
//
//        armdown(robot);
//        sleep(800);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//        sleep(150);
//
//        move(robot, 0.75, 7, 6, RIGHT);
//
//        move(robot, 0.75, 2, 6, FORWARD);
//
//
//    }
//    public void squareB() {
//        move(robot, 0.45, 6, 6, LEFT); //test
//        sleep(100); //test
//
//        move(robot, 0.75, 34, 6, FORWARD);
//        sleep(150);
//
//        move(robot, 0.325, 12, 4, RIGHT); //13
//        sleep(150);
//
//        armdown(robot);
//        sleep(1100);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//
//        move(robot, 0.325, 18.8, 4, TURNRIGHT); //18.3
//        sleep(200);
//
//        armdown(robot);
//
//        move(robot, 0.75, 22, 6, FORWARD);
//        sleep(150);
//
//        move(robot, 0.325, 4.2, 4, TURNRIGHT);
//        sleep(100);
//
//        closeClaw(robot);
//        sleep(1000);
//
//        armup(robot);
//
//        move(robot, 0.325, 23.4, 4, TURNLEFT);
//        sleep(200);
//
//        move(robot, 0.75, 22, 6, FORWARD);
//        sleep(200);
//
//        armdown(robot);
//        sleep(1100);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//        sleep(200);
//
//        move(robot, 0.75, 10, 6, RIGHT);
//        sleep(200);
//
////        move(robot, 0.7, 12, 4, LEFT);
////        sleep(200);
////
////        move(robot, 0.7, 41, 4, BACK);
////        sleep(200);
////
////        move(robot, 0.325, 20, 4, RIGHT);
////        sleep(200);
////
////        closeClaw(robot);
////        sleep(800);
////
////        move(robot, 0.75, 40, 6, FORWARD);
////        sleep(200);
////
////        move(robot, 0.6, 7, 4, LEFT);
////        sleep(100);
////
////        openClaw(robot);
////        sleep(300);
////
////        move(robot, 0.7, 6, 4, BACK);
//
//
//    }
//    public void squareC() {
//        move(robot, 0.75, 45, 6, FORWARD);
//        sleep(150);
//
//        move(robot, 0.45, 6, 6, LEFT); //test
//
//        armdown(robot);
//        sleep(800);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//
//        move(robot, 0.325, 18, 4, TURNRIGHT);//17.8
//        sleep(200);
//
//        armdown(robot);
//
//        move(robot, 0.75, 34.5, 6, FORWARD);
//        sleep(150);
//
//        move(robot, 0.325, 4, 4, TURNRIGHT);
//        sleep(150);
//
//        closeClaw(robot);
//        sleep(1000);
//
//        armup(robot);
//
//        move(robot, 0.325, 16.6, 4, TURNRIGHT);
//        sleep(200);
//
//        move(robot, 0.75, 33.7, 6, FORWARD);
//        sleep(200);
//
//        armdown(robot);
//        sleep(800);
//
//        openClaw(robot);
//        sleep(200);
//
//        armup(robot);
//        sleep(200);
//
//        move(robot, 0.75, 10, 6, BACK);
//        sleep(200);
//
////        move(robot, 0.7, 50, 4, BACK);
////        sleep(100);
////
////        move(robot, 0.325, 18.5, 4, RIGHT);
////        sleep(200);
////
////        closeClaw(robot);
////        sleep(800);
////
////        move(robot, 0.75, 48, 6, FORWARD);
////        sleep(200);
////
////        move(robot, 0.6, 16, 4, LEFT);
////        sleep(100);
////
////        openClaw(robot);
////        sleep(300);
////
////        move(robot, 0.7, 16, 4, BACK);
//
//    }
//}
////        move(robot, 0.325, 11.25, 4, RIGHT);
////        sleep(250);
////
////        move(robot, 0.75, 27, 6, FORWARD);
////        sleep(250);
////
////        label = ob.detectRingLabel();
////
////
////        if(label == null) {
////            move(robot, 0.45, 20, 6, LEFT);
////            sleep(250);
////
////            move(robot, 0.75, 46, 6, FORWARD);
////            sleep(250);
////
////            move(robot, 0.25, 8, 5, BACK);
////        } else if(label.equals("Single")) {
////            move(robot, 1, 72.5, 12, FORWARD);
////            sleep(250);
////
////            move(robot, 0.35, 30, 6, LEFT);
////            sleep(250);
////
////            move(robot, 0.35, 24, 8, BACK);
////        } else if(label.equals("Quad")) {
////            move(robot, 0.45, 25, 6, LEFT);
////            sleep(250);
////
////            move(robot, 0.1, 90, 12, FORWARD);
////            sleep(250);
////
////            move(robot, 0.3, 30, 6, RIGHT);
////            sleep(250);
////
////            move(robot, 0.35, 45, 12, BACK);
////        }