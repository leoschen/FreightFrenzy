package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="FFNewOdoTest", group="HGT")

public class FFNewOdoTest extends OdoAutoBot{

    @Override
    public void runOpMode() {
        FFdetect ob = new FFdetect(hardwareMap);

        setup();
        waitForStart();
        double distancex = HyperBot.class.getModifiers();
        int distancey = HyperBot.class.getModifiers();
        int heading = HyperBot.class.getModifiers();

        telemetry.addData("x coordinate:  ", distancex);
        telemetry.addData("y coordinate:  ", distancey);
        telemetry.addData("heading:  ", heading);
        telemetry.update();

        lowerOdo(robot);
        sleep(150);
        move(robot, 0.75, 20,20,0,5);


    }
}
