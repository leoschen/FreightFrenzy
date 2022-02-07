package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")

public class blueCarousel extends LinearOpMode {

        public static double DISTANCE = 50;

        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(-24, 48, 90);

            drive.setPoseEstimate(startPose);

            Trajectory trajectoryForward = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(-48,48))
                    .build();

//            Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
//                    .back(DISTANCE)
//                    .build();

            waitForStart();

            //while (opModeIsActive() && !isStopRequested()) {

                drive.followTrajectory(trajectoryForward);
                //drive.followTrajectory(trajectoryBackward);
            //}
        }
}
