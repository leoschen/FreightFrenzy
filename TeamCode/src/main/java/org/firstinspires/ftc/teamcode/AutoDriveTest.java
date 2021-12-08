package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AutoDriveTest", group="HGT")

public class AutoDriveTest extends FFGyroAutoBot{
    @Override
    public void runOpMode() {
        boolean warehouseParking = false;
        FFdetect ob = new FFdetect(hardwareMap);
        ob.init();
        String label;

//        label = ob.detectDuckPos();
//        telemetry.addData("# Object Detected:  ", label);
//        telemetry.update();
        setup();

        waitForStart();

        lowerOdo(robot);
        sleep(200);

        label = ob.detectDuckPos();
        telemetry.addData("# Object Detected:  ", label);
        telemetry.update();

        moveWithOdo(robot, 0.75, 12, 6, BACK);
        sleep(150);

        moveWithOdo(robot, 0.75, 21.8, 6, RIGHT);
        sleep(130);

//        if(label.contentEquals("left")) {
        armup1(robot);
        spinUp(robot, 3, 3);
//        } else if(label.contentEquals("middle")) {
//            armup1(robot);
//            spin(robot, 20, 3);
//        } else {
//            armup1(robot);
//        }



        sleep(150);
        armdown(robot, 1900);
        moveWithOdo(robot,0.1, 6.5, 6, TURNLEFT);
        sleep(150);
        moveWithOdo(robot, 0.7, 43.5, 6, FORWARD);
        moveWithOdo(robot, 0.2, 8, 6, FORWARD);
        sleep(150);
        spinCarousel(robot, 3);


        if(warehouseParking){
            moveWithOdo(robot, 1, 3.5, 6, TURNRIGHT);
            moveWithOdo(robot, 0.7, 40, 8, FORWARD);
        }else {
            moveWithOdo(robot, 0.5, 6, 10, TURNLEFT);
            sleep(150);
//            raiseOdo(robot);
//            sleep(150);
            moveWithOdo(robot, 1, 80, 10, FORWARD);


            //tested at 12.4 volts
        }





//        gyroTurn(robot,1,90, LEFT);
//        setup();
//        lowerOdo(robot);
//        waitForStart();

//        moveWithOdo(robot, 1,40, 5, FORWARD);
//        sleep(5000);
//        moveWithOdo(robot, 0.1,20, 5, BACK);
//        sleep(5000);
//        moveWithOdo(robot, 0.1,20, 5, LEFT);
//        sleep(5000);
//        moveWithOdo(robot, 0.1,20, 5, RIGHT);
//
//        sleep(5000);
//        gyroTurn(robot, 0.1, 90, LEFT);
//        sleep(5000);
//        gyroTurn(robot, 0.1, 90, RIGHT);
    }
}
