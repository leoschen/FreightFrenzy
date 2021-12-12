package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="HyperBotDriver", group="HGT")
public class HyperBotDriver extends LinearOpMode {
    //has motor and servo info
    HyperBot robot = new HyperBot();

    private static final double CLAW_SPEED = 0.25;  // sets rate to move servo


    double sidewayRightX    = 0;
    double forwardY         = 0;
    double turn             = 0;

    double frontLeftPower   = 0;
    double frontRightPower  = 0;
    double backLeftPower    = 0;
    double backRightPower   = 0;
    double switchBack       = 1;
    int target              = 0;

    double spinnerPower     = 1;
    double suckPower        = 0;
    double intakePower      = 0;
    double shootPower       = 0;
    double shooterPower     = 0.9;
    double intakeArmPower   = 0;
    double armPower         = 1;

    double clawOffset = 0;
    double pusherOffset = 1;
    double armOffset = 1;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        //robot config
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            robot.linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.linearDrive.getCurrentPosition();
            double linearPower = gamepad2.right_stick_y * -1;
//            robot.linearDrive.setPower(linearPower*0.4);

            //driving robot
//            sidewayRightX = gamepad1.left_stick_x; // left stick right(1)/left(-1)
//            forwardY = gamepad1.left_stick_y * -1; // left stick forward(-1)/back(1)
//            turn  =  -gamepad1.right_stick_x;


            robot.spinner.setPower(spinnerPower*gamepad2.right_stick_y*0.3);
            if(gamepad1.left_stick_x > -0.85 && gamepad1.left_stick_x < 0) {
                sidewayRightX = 0.35 * gamepad1.left_stick_x;
            } else if(gamepad1.left_stick_x < 0.85 && gamepad1.left_stick_x > 0) {
                sidewayRightX = 0.35 * gamepad1.left_stick_x;
            } else {
                sidewayRightX = gamepad1.left_stick_x;
            }

            if(gamepad1.left_stick_y > -0.85 && gamepad1.left_stick_y < 0) {
                forwardY = 0.35 * gamepad1.left_stick_y * -1;
            } else if(gamepad1.left_stick_y < 0.85 && gamepad1.left_stick_y > 0) {
                forwardY = 0.35 * gamepad1.left_stick_y * -1;
            } else {
                forwardY = gamepad1.left_stick_y * -1;
            }

            if(gamepad1.right_stick_x > -0.85 && gamepad1.right_stick_x < 0) {
                turn  =  0.35 * gamepad1.right_stick_x;
            } else if(gamepad1.right_stick_x < 0.85 && gamepad1.right_stick_x > 0) {
                turn  =  0.35 * gamepad1.right_stick_x;
            } else {
                turn  =  gamepad1.right_stick_x;
            }


            //moving and toggling

            if (gamepad1.left_bumper) {
                switchBack = -1;
            }
            if (gamepad1.right_bumper) {
                switchBack = 1;
            }

            if (switchBack == 1) {
                frontLeftPower = Range.clip(forwardY + sidewayRightX + turn, -1.0, 1.0);
                frontRightPower = Range.clip(forwardY - sidewayRightX - turn, -1.0, 1.0);
                backLeftPower = Range.clip(forwardY - sidewayRightX + turn, -1.0, 1.0);
                backRightPower = Range.clip(forwardY + sidewayRightX - turn, -1.0, 1.0);
            } else if (switchBack == -1) {
                frontLeftPower = Range.clip(-forwardY - sidewayRightX - turn, -1.0, 1.0);
                frontRightPower = Range.clip(-forwardY + sidewayRightX + turn, -1.0, 1.0);
                backLeftPower = Range.clip(-forwardY + sidewayRightX - turn, -1.0, 1.0);
                backRightPower = Range.clip(-forwardY - sidewayRightX + turn, -1.0, 1.0);
            }

            robot.frontLeft.setPower(frontLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backLeft.setPower(backLeftPower);
            robot.backRight.setPower(backRightPower);

//            // the position 1 is when the claw is closed and 0 is open so to open the claw it subtracts
            if (gamepad1.dpad_up) {
                clawOffset = 0;
            } else if (gamepad1.dpad_down) {
                clawOffset = 1;
            }
            if (gamepad2.dpad_up) {
                robot.clawServo.setPosition(0);
            } else if (gamepad2.dpad_down) {
                robot.clawServo.setPosition(1);
            }
            clawOffset = Range.clip(clawOffset, 0, 1);
            robot.leftServo.setPosition(clawOffset);
            robot.backServo.setPosition(clawOffset);
            robot.rightServo.setPosition(1-clawOffset);
//
//            //arm for wobble grabber
//
//            if (gamepad2.left_bumper) {
//                robot.shooterLeft.setPower(shooterPower);
//                robot.shooterRight.setPower(shooterPower);

//                target = robot.armMotor.getCurrentPosition()-1700;
//                robot.armMotor.setTargetPosition(target);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.armMotor.setPower(1);
//                while(robot.armMotor.isBusy()) {
//                    if(robot.armMotor.getCurrentPosition()==target) {
//                        robot.armMotor.setPower(0);
//                        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        break;
//                    }
//                }

////                armOffset += CLAW_SPEED;//delete -0.1  if the servo is going to slow
//            } else if (gamepad2.right_bumper) {
//                robot.shooterLeft.setPower(-shooterPower);
//                robot.shooterRight.setPower(-shooterPower);
//                target = robot.armMotor.getCurrentPosition()+1700;
//                robot.armMotor.setTargetPosition(target);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.armMotor.setPower(-1);
//                while(robot.armMotor.isBusy()) {
//                    if(robot.armMotor.getCurrentPosition()==target) {
//                        robot.armMotor.setPower(0);
//                        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        break;
//                    }
//                }
//                armOffset -= CLAW_SPEED-0.2;//delete -0.1 if the servo is going to slow

                 // Edit here for shooter power
                 // * a smaller number for less power
//            } else if (gamepad2.a) {
//                robot.shooterLeft.setPower(shooterPower*-0.9);//change 0.9
//                robot.shooterRight.setPower(shooterPower*-0.9);//change 0.9
//            } else if (!gamepad2.left_bumper) {
//                robot.shooterLeft.setPower(0);
//                robot.shooterRight.setPower(0);
//            } else if (!gamepad2.right_bumper) {
//                robot.shooterLeft.setPower(0);
//                robot.shooterRight.setPower(0);
//            } else if (!gamepad2.a) {
//                robot.shooterLeft.setPower(0);
//                robot.shooterRight.setPower(0);
//            }


//            if(gamepad1.a) {
//                // Turn off RUN_TO_POSITION
//                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                double nOffset = angles.firstAngle;
//                if(nOffset >= 170) {nOffset = -nOffset + 360;}
//
//                if (nOffset == 0) {
//                }else if (nOffset > 0) {
//                    robot.frontLeft.setPower(0.3);
//                    robot.frontRight.setPower(-0.3);
//                    robot.backLeft.setPower(0.3);
//                    robot.backRight.setPower(-0.3);
//                    while(nOffset > 0) {
//                        sleep(6);
//                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                        nOffset = angles.firstAngle;
//                        if(nOffset >= 170) {nOffset = -nOffset + 360;}
//                        telemetry.addData("right:", angles.firstAngle);
//                        telemetry.update();
//                        if(nOffset < 0){
//                            break;
//                        }
//                    }
//                } else {
//                    robot.frontLeft.setPower(-0.3);
//                    robot.frontRight.setPower(0.3);
//                    robot.backLeft.setPower(-0.3);
//                    robot.backRight.setPower(0.3);
//                    while(nOffset < 0) {
//                        sleep(6);
//                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                        nOffset = angles.firstAngle;
//                        if(nOffset >= 170) {nOffset = -nOffset + 360;}
//                        telemetry.addData("left:", angles.firstAngle);
//                        telemetry.update();
//                        if(nOffset > 0){
//                            break;
//                        }
//                    }
//                }
//
//                // Stop all motion;
//                robot.frontLeft.setPower(0);
//                robot.frontRight.setPower(0);
//                robot.backLeft.setPower(0);
//                robot.backRight.setPower(0);
//            }



//            armOffset = Range.clip(armOffset, 0.24, 0.94);//change this lower if you want the bar to go up less
//            robot.clawArm.setPosition(armOffset);

            //intake

            if (gamepad2.left_trigger != 0) {
                suckPower = gamepad2.left_trigger;
                robot.sucker.setPower(suckPower);
            } else {
                suckPower = - gamepad2.right_trigger * 0.5;
            robot.sucker.setPower(suckPower);
            }
//            robot.intakeRight.setPower(intakePower);


//            robot.intakeRight.setPower(intakePower)
        robot.armMotor.setPower(gamepad2.left_stick_y * -armPower);



//            intakePower = gamepad2.left_trigger;
////            robot.intakeLeft.setPower(intakePower);
////            robot.intakeRight.setPower(intakePower);
//
//            intakePower = 0 - gamepad2.right_trigger * shooterPower;
////            robot.intakeLeft.setPower(intakePower);
////            robot.intakeRight.setPower(intakePower);
//
//            intakeArmPower = gamepad2.left_stick_y*0.75;
//            robot.intakeArm.setPower(intakeArmPower);
//            robot.intakeArm.setPower(0.75);
//            if (gamepad2.a) {
//                robot.intakeArm.setTargetPosition(robot.intakeArm.getCurrentPosition()+350);
//            }


//            armPower = gamepad2.right_stick_y*0.75;
//            robot.armMotor.setPower(-armPower);

//            shootPower = gamepad2.right_stick_y*0.75;
//            robot.shooterLeft.setPower(shootPower);
//            robot.shooterRight.setPower(shootPower);

//            telemetry.addData("armMotor position: ", robot.armMotor.getCurrentPosition());
//            telemetry.addData("pusher position: ", pusherOffset);

            //debugging
            telemetry.addData("Motors", "FL(%.2f), FR(%.2f), BL:(%.2f), BR:(%.2f), SPNR:(%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower, robot.spinner.getPower()
//                    , (double)robot.bEncoder.getCurrentPosition()*0.0134, (double)robot.lEncoder.getCurrentPosition()*0.0134, (double)robot.rEncoder.getCurrentPosition()*0.0134
            );
            telemetry.addData("Encoders", "L:(%d), B: (%d), R: (%d)",
                    robot.sucker.getCurrentPosition(),robot.spinner.getCurrentPosition(),robot.rEncoder.getCurrentPosition());
//            , , robot.rEncoder.getCurrentPosition()
//            telemetry.addData("Servos", "Claw(%.2f), Linear:(%d), ",
//                    clawOffset, robot.linearDrive.getCurrentPosition());

//            telemetry.addData("Sensors", "Distance(%.2f in), Red(%d), Green(%d), Blue(%d)",
//                    robot.distanceSensor.getDistance(DistanceUnit.INCH),
//                    robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());

            telemetry.update();
        }
    }
}

