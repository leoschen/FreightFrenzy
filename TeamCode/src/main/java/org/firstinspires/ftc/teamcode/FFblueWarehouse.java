package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="FFblueWarehouse", group="HGT")

public class FFblueWarehouse extends AutoBot{
    @Override
    public void runOpMode() {
        boolean warehouseParking = true;
        FFdetect ob = new FFdetect(hardwareMap);
        ob.init();
        String label;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14,48,90);

        drive.setPoseEstimate(startPose);
//        label = ob.detectDuckPos();
//        telemetry.addData("# Object Detected:  ", label);
//        telemetry.update();
        setup();
        waitForStart();
        double distancex = HyperBot.class.getModifiers();
        int distancey = HyperBot.class.getModifiers();
        int heading = HyperBot.class.getModifiers();

        label = ob.detectDuckPos();
        telemetry.addData("# Object Detected:  ", label);
        telemetry.addData("x coordinate:  ", distancex);
        telemetry.addData("y coordinate:  ", distancey);
        telemetry.addData("heading:  ", heading);
        telemetry.update();


        TrajectorySequence strafe = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(5.5)
                .forward(-30)
                .turn(Math.toRadians(-90))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(strafe);

        sleep(250);
        armup1(robot);
        sleep(250);

        TrajectorySequence move = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))//counter clockwise
                .back(-25)
                .strafeRight(11.5)
                .forward(-50)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(move);












    }
}
