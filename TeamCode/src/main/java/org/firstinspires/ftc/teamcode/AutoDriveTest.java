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

        moveWithOdo(robot, 0.75, 11.5, 6, BACK);//12.5
        sleep(150);

        moveWithOdo(robot, 0.75, 21.3, 6, RIGHT);
        sleep(130);

        if(label.contentEquals("left")) {
            //bottom
            moveWithOdo(robot, 0.75, 3, 6, FORWARD);
            sleep(150);


            armup3(robot);
            sleep(150);
            drop(robot);
            sleep(1000);
            close(robot);

            sleep(150);
            armdown(robot, 2350);

            moveWithOdo(robot,0.3, 19.5, 6, TURNLEFT);
            sleep(150);

            moveWithOdo(robot, 0.7, 37, 6, FORWARD);
            moveWithOdo(robot, 0.2, 6, 6, FORWARD);

        } else if(label.contentEquals("middle")) {
            //middle
            moveWithOdo(robot, 0.75, 2, 6, FORWARD);
            sleep(150);


            armup2(robot);
            sleep(150);
            drop(robot);
            sleep(1000);
            close(robot);

            sleep(150);
            armdown(robot, 2100);

            moveWithOdo(robot,0.3, 19.3, 6, TURNLEFT);
            sleep(150);

            moveWithOdo(robot, 0.7, 40, 6, FORWARD);
            moveWithOdo(robot, 0.2, 7, 6, FORWARD);

        } else {
            //top
            moveWithOdo(robot, 0.75, 2, 6, BACK);
            sleep(150);

            armup1(robot);
            lower(robot);
            sleep(150);
            drop(robot);
            sleep(1000);
            close(robot);

            sleep(150);
            armdown(robot, 1500);

            moveWithOdo(robot,0.3, 18.5, 6, TURNLEFT);
            sleep(150);

            moveWithOdo(robot, 0.7, 43, 6, FORWARD);
            moveWithOdo(robot, 0.2, 7, 6, FORWARD);

        }






        sleep(150);
        spinCarouselTime(robot, 3);


        if(warehouseParking){
            moveWithOdo(robot, 1, 17, 6, TURNRIGHT);
            moveWithOdo(robot, 0.7, 40, 8, FORWARD);
        }else {
            raiseOdo(robot);
            if(label.contentEquals("left")) {
                moveWithOdo(robot, 0.5, 43.7, 10, TURNLEFT);
            } else if (label.contentEquals("middle")) {
                moveWithOdo(robot, 0.5, 41.3, 10, TURNLEFT);
            } else {
                moveWithOdo(robot, 0.5, 44, 10, TURNLEFT);
            }
            sleep(150);
//            raiseOdo(robot);
//            sleep(150);
            moveWithOdo(robot, 1, 120, 15, FORWARD);


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
