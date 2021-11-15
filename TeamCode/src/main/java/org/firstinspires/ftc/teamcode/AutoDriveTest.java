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

@Autonomous(name="AutoDriveTest", group="HGT")

public class AutoDriveTest extends FFGyroAutoBot{
    @Override
    public void runOpMode() {

        setup();
        lowerOdo(robot);
        waitForStart();

        moveWithOdo(robot, 0.1,20, 5, FORWARD);
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
