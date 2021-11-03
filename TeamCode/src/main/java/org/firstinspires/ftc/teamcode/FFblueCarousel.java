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

@Autonomous(name="FFblueCarousel", group="HGT")

public class FFblueCarousel extends AutoBot{
    @Override
    public void runOpMode() {
        boolean warehouseParking = true;
        FFdetect ob = new FFdetect(hardwareMap);
        ob.init();
        String label;

//        label = ob.detectDuckPos();
//        telemetry.addData("# Object Detected:  ", label);
//        telemetry.update();
        setup();
        waitForStart();

        label = ob.detectDuckPos();
        telemetry.addData("# Object Detected:  ", label);
        telemetry.update();


        move(robot, 0.75, 22.3, 6, RIGHT);
        sleep(130);

        move(robot, 0.45, 12.6, 6, BACK);
        sleep(150);

        if(label.contentEquals("left")) {
            armup1(robot);
        } else if(label.contentEquals("middle")) {
            armup1(robot);
        } else {
            armup1(robot);
        }

        spin(robot, 20, 3);

        sleep(150);
        armdown(robot, 2400);
        move(robot,0.5, 18.4, 6, TURNLEFT);
        sleep(150);
        move(robot, 0.7, 43, 6, FORWARD);
        move(robot, 0.2, 8, 6, FORWARD);
        sleep(150);
        spinCarousel(robot, 3);


        if(warehouseParking){
            move(robot, 0.5, 39, 6, TURNRIGHT);
            move(robot, 0.7, 80, 8, FORWARD);
        }else {
            move(robot, 0.5, 24.4, 6, TURNLEFT);
            sleep(150);
            move(robot, 0.7, 17, 6, FORWARD);
            //tested at 12.4 volts
        }



    }
}
