package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="OdoAutoBot", group="HGT")
@Disabled
public class OdoAutoBot extends LinearOpMode {
    HyperBot           robot = new HyperBot();
    ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_ROTATION_HD_HEX_MOTOR = 537.7;
    static final double     WHEEL_DIAMETER_INCHES  = 3.78 ;     // 96mm
    static final double     COUNTS_PER_INCH = COUNTS_PER_ROTATION_HD_HEX_MOTOR / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        // do nothing here. All classes that extends AutoBot need to override this method
    }

    public void setup() {
        robot.init(hardwareMap);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        robot.clawServo.setPosition(0);
    }

    public void move(HyperBot robot, double speed, double targetX, double targetY, double targetH, int timeoutS) {
        double distanceX = targetX - robot.getCurrentX();
        double distanceY = targetY - robot.getCurrentY();
        double direction = targetH - robot.getCurrentH();
        double startX = targetX - robot.getCurrentX();
        double startY = targetY - robot.getCurrentY();
        double startH = targetH - robot.getCurrentH();
        boolean reachedX = false;
        boolean reachedY = false;
        boolean reachedH = false;
        double X = 1;
        double Y = 1;
        double H = 1;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (!reachedX && !reachedY && !reachedH)) {
                sleep(50);
                distanceX = targetX - robot.getCurrentX();
                distanceY = targetY - robot.getCurrentY();
                direction = targetH - robot.getCurrentH();

                X = startX - robot.getCurrentX() + distanceX;
                Y = startY - robot.getCurrentY() + distanceY;
                H = startH - robot.getCurrentH() + direction;
                if (X < startX - robot.getCurrentX()) {
                    reachedX = true;
                } else if (Y < startY - robot.getCurrentY()) {
                    reachedY = true;
                } else if (H < startH - robot.getCurrentH()) {
                    reachedH = true;
                }

                double maximum = Math.max(Math.max(distanceY +distanceX +direction, distanceY -distanceX -direction) , Math.max(distanceY -distanceX +direction, distanceY +distanceX -direction));
                robot.frontLeft.setPower( ((distanceY +distanceX +direction) / maximum) * speed);
                robot.frontRight.setPower(((distanceY -distanceX -direction) / maximum) * speed);
                robot.backLeft.setPower(  ((distanceY -distanceX +direction) / maximum) * speed);
                robot.backRight.setPower( ((distanceY +distanceX -direction) / maximum) * speed);


                telemetry.addData("position:  ","FL(%d)", robot.frontLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }



    public void spin (HyperBot robot, double rotations, double timeoutS) {
        //start spinning
        int sTarget = robot.frontLeft.getCurrentPosition() + (int) (rotations * COUNTS_PER_INCH);
        robot.sucker.setTargetPosition(sTarget);
        robot.sucker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.sucker.setPower(1);
        //wait until we reach the target
        while (opModeIsActive() && runtime.seconds() < timeoutS && robot.sucker.isBusy()) {
            sleep(50);
        }
        //stop motion
        robot.sucker.setPower(0);
        robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    int target = 0;


    public void armup1(HyperBot robot) {
        target = robot.armMotorLeft.getCurrentPosition() - 2500;
        robot.armMotorLeft.setTargetPosition(target);
        robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorLeft.setPower(1);
        while (robot.armMotorLeft.isBusy()) {
            telemetry.addData("armPos","(%d)", robot.armMotorLeft.getCurrentPosition());
            telemetry.update();
            if (robot.armMotorLeft.getCurrentPosition() <= target) {
                robot.armMotorLeft.setPower(0);
                robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }
    public void armup2(HyperBot robot) {
        target = robot.armMotorLeft.getCurrentPosition() - 650;
        robot.armMotorLeft.setTargetPosition(target);
        robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorLeft.setPower(1);
        while (robot.armMotorLeft.isBusy()) {
            if (robot.armMotorLeft.getCurrentPosition() <= target) {
                robot.armMotorLeft.setPower(0);
                robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }
    public void armup3(HyperBot robot) {
        target = robot.armMotorLeft.getCurrentPosition() - 1200;
        robot.armMotorLeft.setTargetPosition(target);
        robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorLeft.setPower(1);
        while (robot.armMotorLeft.isBusy()) {
            if (robot.armMotorLeft.getCurrentPosition() <= target) {
                robot.armMotorLeft.setPower(0);
                robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }

    public void armdown (HyperBot robot, int distance) {
        target = robot.armMotorLeft.getCurrentPosition()+distance;
        robot.armMotorLeft.setTargetPosition(target);
        robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorLeft.setPower(-1);
        while(robot.armMotorLeft.isBusy()) {
            if(robot.armMotorLeft.getCurrentPosition()>=target) {
                robot.armMotorLeft.setPower(0);
                robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }

    public void spinCarousel (HyperBot robot, double timeoutS) {
        target = robot.spinner.getCurrentPosition()-4000;
        robot.spinner.setTargetPosition(target);
        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.spinner.setPower(0.06);
        while(robot.spinner.isBusy() && runtime.seconds() < timeoutS) {
            if(robot.spinner.getCurrentPosition()<=target) {
                robot.spinner.setPower(0);
                robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }


    public void lowerOdo(HyperBot robot) {
        robot.leftServo.setPosition(1);
        robot.rightServo.setPosition(0);
        robot.backServo.setPosition(1);
    }
}
