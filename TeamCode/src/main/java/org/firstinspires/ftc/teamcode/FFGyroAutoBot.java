package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="FFGyroAutoBot", group="HGT")
@Disabled
public class FFGyroAutoBot extends LinearOpMode {


    HyperBot robot = new HyperBot();

    ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_ROTATION_HD_HEX_MOTOR = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96mm
    static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION_HD_HEX_MOTOR / (WHEEL_DIAMETER_INCHES * 3.1415);
//8192
    static final double ODOMETER_COUNTS_PER_INCH = (1.37795 * 3.1415926) / 8192;

    static final int FORWARD = 1;
    static final int BACK = 2;
    static final int RIGHT = 3;
    static final int LEFT = 4;
    static final int TURNLEFT = 5;
    static final int TURNRIGHT = 6;

    boolean movingForward = true;
    double turn;
    double offset = 0;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    int leftOdometry;
    int rightOdometry;
    int middleOdometry;

    int left;
    int right;
    int middle;

    double leftDistance;
    double rightDistance;
    int middleDistance;

    static final double PIDDistance = 7;
    double PIDSpeed = 1;

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
        robot.armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




//        robot.clawServo.setPosition(0);
    }

    public void moveWithOdo(HyperBot robot, double speed, double inches, int timeoutS, int direction) {

        System.out.println("\n\n\n\nstart\n\n\n\n\n");
        telemetry.addData("start moving with Odometer" , "");
        telemetry.update();

        int frontLeftTarget = 0;
        int frontRightTarget = 0;
        int backLeftTarget = 0;
        int backRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if (direction == FORWARD) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == BACK) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            } else if (direction == RIGHT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == LEFT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            }

            if (direction == TURNLEFT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == TURNRIGHT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            }

            if (direction == TURNLEFT || direction == TURNRIGHT) {
//                robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.rEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                leftDistance = robot.sucker.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                rightDistance = -robot.rEncoder.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                double starting = 0;
//                if (direction == TURNLEFT) {
//                    starting = rightDistance;
//                } else if (direction == TURNRIGHT) {
//                    starting = leftDistance;
//                }
//                double startAngle = leftDistance - rightDistance;
//                correction = checkTurnOdo(direction, starting);
//
//                if (direction == TURNLEFT) {
//                    while(rightDistance + startAngle < starting + inches) {
//                        leftDistance = robot.sucker.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                        rightDistance = -robot.rEncoder.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                        correction = 0;
//                        robot.frontLeft.setPower(-speed - correction * 0.04);
//                        robot.frontRight.setPower(speed - correction * 0.04);
//                        robot.backLeft.setPower(-speed - correction * 0.04);
//                        robot.backRight.setPower(speed - correction * 0.04);
////                        correction = checkTurnOdo(direction, starting);
//                        sleep(50);
//                        telemetry.addData("position:  ", "FL(%d), right distance(%2f), starting(%2f)", robot.frontLeft.getCurrentPosition(), rightDistance, startAngle);
//                        telemetry.update();
//                    }
//                } else if (direction == TURNRIGHT) {
//                    while(leftDistance + startAngle < starting + inches) {
//                        leftDistance = robot.sucker.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                        rightDistance = -robot.rEncoder.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
//                        correction = 0;
//                        robot.frontLeft.setPower(speed - correction * 0.04);
//                        robot.frontRight.setPower(-speed - correction * 0.04);
//                        robot.backLeft.setPower(speed - correction * 0.04);
//                        robot.backRight.setPower(-speed - correction * 0.04);
////                        correction = checkTurnOdo(direction, starting);
//                        sleep(50);
//                        telemetry.addData("position:  ", "FL(%d), FLTarget(%d)", robot.frontLeft.getCurrentPosition(), frontLeftTarget);
//                        telemetry.update();
//                }



                robot.frontLeft.setTargetPosition(frontLeftTarget);
                robot.frontRight.setTargetPosition(frontRightTarget);
                robot.backLeft.setTargetPosition(backLeftTarget);
                robot.backRight.setTargetPosition(backRightTarget);

                // Turn On RUN_TO_POSITION
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.frontLeft.setPower(Math.abs(speed));
                robot.frontRight.setPower(Math.abs(speed));
                robot.backLeft.setPower(Math.abs(speed));
                robot.backRight.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                    sleep(50);
                    telemetry.addData("position:  ", "FL(%d), FLTarget(%d)", robot.frontLeft.getCurrentPosition(), frontLeftTarget);
                    telemetry.update();
                }


                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else {

                robot.frontLeft.setTargetPosition(frontLeftTarget);
                robot.frontRight.setTargetPosition(frontRightTarget);
                robot.backLeft.setTargetPosition(backLeftTarget);
                robot.backRight.setTargetPosition(backRightTarget);

                // Turn On RUN_TO_POSITION
                System.out.println("front left target: " + robot.frontLeft.getTargetPosition() +
                        "\nfront right target: " + robot.frontRight.getTargetPosition() +
                        "\nback left target: " + robot.backLeft.getTargetPosition() +
                        "\nback right target: " + robot.backRight.getTargetPosition());

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(6);

                correction = 0;
                left = robot.sucker.getCurrentPosition();
                right = -robot.armMotorLeft.getCurrentPosition();
                middle = robot.spinner.getCurrentPosition();

                leftDistance = robot.sucker.getCurrentPosition() + (inches * ODOMETER_COUNTS_PER_INCH);
                rightDistance = -robot.armMotorLeft.getCurrentPosition() + (inches * ODOMETER_COUNTS_PER_INCH);

                System.out.println("\n\n\n\nTarget set and encoders distance set\n\n\n\n\n");
                telemetry.addData("Target set and encoders distance set", "");
                telemetry.update();
                runtime.reset();
                do {
                    correction = 0;
                    if (direction == TURNRIGHT || direction == TURNLEFT) {
                        break;
                    } else {
                        correction = checkOdometry();
                        System.out.println("\n\n\n\ncheck straight line\n\n\n\n");

                    }


                    telemetry.addData("correction : ", correction);
                    telemetry.update();


                    if (direction == RIGHT || direction == BACK) {
                        System.out.println("\n\n\n\nset speed\n\n\n\n");
//PID(robot.sucker.getCurrentPosition(), robot.rEncoder.getCurrentPosition(), left, right, inches, speed);

                        PIDSpeed = PID(robot.sucker.getCurrentPosition(), -robot.armMotorLeft.getCurrentPosition(), left, right, inches, speed);

                        robot.frontLeft.setPower(Math.abs(PIDSpeed) - (correction * 0.4));//0.6
                        robot.frontRight.setPower(Math.abs(PIDSpeed) + (correction * 0.4));//0.6
                        robot.backLeft.setPower(Math.abs(PIDSpeed) - (correction * 0.4));//0.6
                        robot.backRight.setPower(Math.abs(PIDSpeed) + (correction * 0.4));//0.6
                        System.out.println("front left power : " + (Math.abs(PIDSpeed) - (correction * 0.4)) + "\ncorrection : " + correction);
                        System.out.println("front right power : " + (Math.abs(PIDSpeed) + (correction * 0.4)) + "\ncorrection : " + correction);
                        System.out.println("PID speed : " + PIDSpeed + "\nspeed : " + speed);
                    } else if (direction == LEFT || direction == FORWARD) {
                        System.out.println("\n\n\n\nset speed\n\n\n\n");

                        PIDSpeed = PID(robot.sucker.getCurrentPosition(), -robot.armMotorLeft.getCurrentPosition(), left, right, inches, speed);

                        robot.frontLeft.setPower(Math.abs(PIDSpeed) + (correction * 0.4));//0.6
                        robot.frontRight.setPower(Math.abs(PIDSpeed) - (correction * 0.4));//0.6
                        robot.backLeft.setPower(Math.abs(PIDSpeed) + (correction * 0.4));//0.6
                        robot.backRight.setPower(Math.abs(PIDSpeed) - (correction * 0.4));//0.6
                        System.out.println("front left power : " + (Math.abs(PIDSpeed) + (correction * 0.4)) + "\ncorrection : " + correction);
                        System.out.println("front right power : " + (Math.abs(PIDSpeed) - (correction * 0.4)) + "\ncorrection : " + correction);
                        System.out.println("PID speed : " + PIDSpeed + "\nspeed : " + speed);

                    }

                    System.out.println("timeout seconds : " + runtime.seconds());


                } while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()));
                System.out.println("\n\n\n\nstop\n\n\n\n");

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public double checkOdometry() {
        System.out.println("\n\n\n\ncheck current position\n\n\n\n----------------------------------------------");
        leftOdometry = robot.sucker.getCurrentPosition() - left;
        rightOdometry = -robot.armMotorLeft.getCurrentPosition() - right;
        middleOdometry = robot.spinner.getCurrentPosition() - middle;

        System.out.println("left" + leftOdometry * ODOMETER_COUNTS_PER_INCH + "\nright" + rightOdometry * ODOMETER_COUNTS_PER_INCH + "\nback" + middleOdometry * ODOMETER_COUNTS_PER_INCH);
        double correct = 0;
        correct = (rightOdometry * ODOMETER_COUNTS_PER_INCH) - (leftOdometry * ODOMETER_COUNTS_PER_INCH);
        System.out.println("\n\n\n\nfind correction\n\n\n\n");
        if (middleOdometry > 0) {
            System.out.println("greater" + correct);
            correct = correct + (middleOdometry * ODOMETER_COUNTS_PER_INCH);
            System.out.println(correct);
        } else if (middleOdometry < 0) {
            System.out.println("fewer" + correct);
            correct = correct - (middleOdometry * ODOMETER_COUNTS_PER_INCH);
            System.out.println(correct);
        }
        System.out.println("middleOdometry : " + middleOdometry * ODOMETER_COUNTS_PER_INCH + "correction : " + correct + "\n--------------------------------------");
        return correct;
    }


    public double checkTurnOdo(int direction, double start) {

        double leftOdom = robot.sucker.getCurrentPosition() * ODOMETER_COUNTS_PER_INCH;
        double rightOdom = -robot.armMotorLeft.getCurrentPosition()  * ODOMETER_COUNTS_PER_INCH;
        double change = rightOdom + leftOdom - start;
//        if(direction == TURNLEFT) {
//            change = rightOdom + leftOdom;
//        } else if (direction == TURNRIGHT) {
//            change = leftOdom + rightOdom;
//        } else {
//            change = 0;
//        }
        return change;
    }

    public void move(HyperBot robot, double speed, double inches, int timeoutS, int direction) {
        moveInternal(robot, speed, inches, timeoutS, direction, -1);
    }

    public void moveWithDistanceSensor(HyperBot robot, double speed, double inches, int timeoutS, int direction, double minDistance) {
        moveInternal(robot, speed, inches, timeoutS, direction, minDistance);
    }

    private void moveInternal(HyperBot robot, double speed, double inches, int timeoutS, int direction, double minDistance) {
        int frontLeftTarget = 0;
        int frontRightTarget = 0;
        int backLeftTarget = 0;
        int backRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if (direction == FORWARD) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == BACK) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            } else if (direction == RIGHT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == LEFT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            } else if (direction == TURNLEFT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            } else if (direction == TURNRIGHT) {
                // Determine new target position, and pass to motor controller
                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            }


            robot.frontLeft.setTargetPosition(frontLeftTarget);
            robot.frontRight.setTargetPosition(frontRightTarget);
            robot.backLeft.setTargetPosition(backLeftTarget);
            robot.backRight.setTargetPosition(backRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            do {
                // get the direction to correct
                correction = checkDirection();

                telemetry.addData("1 imu heading:", lastAngles.firstAngle);
                telemetry.addData("2 global heading:", globalAngle);
                telemetry.addData("3 correction:", correction);
                telemetry.update();

                //move it in the right direction and add the correction adjustment to the speed
                if (direction == RIGHT || direction == BACK) {
                    robot.frontLeft.setPower(Math.abs(speed) - (correction * 0.6));//0.6
                    robot.frontRight.setPower(Math.abs(speed) + (correction * 0.6));//0.6
                    robot.backLeft.setPower(Math.abs(speed) - (correction * 0.6));//0.6
                    robot.backRight.setPower(Math.abs(speed) + (correction * 0.6));//0.6
                } else if (direction == LEFT || direction == FORWARD) {
                    robot.frontLeft.setPower(Math.abs(speed) + (correction * 0.6));//0.6
                    robot.frontRight.setPower(Math.abs(speed) - (correction * 0.6));//0.6
                    robot.backLeft.setPower(Math.abs(speed) + (correction * 0.6));//0.6
                    robot.backRight.setPower(Math.abs(speed) - (correction * 0.6));//0.6
                }

                sleep(6);

            } while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()));

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double nOffset = angles.firstAngle;
            if(nOffset >= 170) {nOffset = -nOffset + 360;}

            if (nOffset == offset) {

            }else if (nOffset - offset > 0) {
                robot.frontLeft.setPower(0.3);
                robot.frontRight.setPower(-0.3);
                robot.backLeft.setPower(0.3);
                robot.backRight.setPower(-0.3);
                while(nOffset - offset > 0) {
                    sleep(6);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    nOffset = angles.firstAngle;
                    if(nOffset >= 170) {nOffset = -nOffset + 360;}
                    telemetry.addData("right:", angles.firstAngle);
                    telemetry.update();
                    if(nOffset - offset < 0){
                        break;
                    }
                }
            } else {
                robot.frontLeft.setPower(-0.3);
                robot.frontRight.setPower(0.3);
                robot.backLeft.setPower(-0.3);
                robot.backRight.setPower(0.3);
                while(nOffset - offset < 0) {
                    sleep(6);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    nOffset = angles.firstAngle;
                    if(nOffset >= 170) {nOffset = -nOffset + 360;}
                    telemetry.addData("left:", angles.firstAngle);
                    telemetry.update();
                    if(nOffset - offset > 0){
                        break;
                    }
                }
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);



        }


    }

    public void gyroTurn (HyperBot robot, double power, double degrees, int direction){

        double  leftPower, rightPower;
        double  targetAngle;
        double  targetLength;
        double  currentOffset;
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // set offset/the new angle to go on

        offset = angles.firstAngle;
        if(offset >= 170) {offset = -offset + 360;}

        // Gyro returns 0->359 when rotating counter clockwise (left) and 359->0 when rotating
        // clockwise (right).

        if (direction == RIGHT) {   // turn right.
            leftPower = power;
            rightPower = -power;
            targetAngle = 360 - degrees + offset;    // degrees is - for right turn.
            targetLength = offset - degrees;
        } else if (direction == LEFT) {   // turn left.
            leftPower = -power;
            rightPower = power;
            targetAngle = degrees + offset;
            targetLength = offset + degrees;
        } else return;

        // set power to rotate.
        robot.frontLeft.setPower(leftPower);
        robot.frontRight.setPower(rightPower);
        robot.backLeft.setPower(leftPower);
        robot.backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (direction == RIGHT)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && angles.firstAngle == 0) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentOffset = angles.firstAngle;
                if(currentOffset >= 170) {currentOffset = -currentOffset + 360;}

                if(targetLength - currentOffset < -10){
                    robot.frontLeft.setPower(leftPower/2*(targetLength - currentOffset));
                    robot.frontRight.setPower(rightPower/2*(targetLength - currentOffset));
                    robot.backLeft.setPower(leftPower/2*(targetLength - currentOffset));
                    robot.backRight.setPower(rightPower/2*(targetLength - currentOffset));
                }

                telemetry.addData("gyro heading", angles.firstAngle);
                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
                telemetry.update();
                idle();
            }

            while (opModeIsActive() && angles.firstAngle > targetLength) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentOffset = angles.firstAngle;
                if(currentOffset >=170) {currentOffset = -currentOffset + 360;}

//                if(degrees + currentOffset < 10){
//                    robot.frontLeft.setPower(leftPower/2*(degrees + currentOffset));
//                    robot.frontRight.setPower(rightPower/2*(degrees + currentOffset));
//                    robot.backLeft.setPower(leftPower/2*(degrees + currentOffset));
//                    robot.backRight.setPower(rightPower/2*(degrees + currentOffset));
//                }

                telemetry.addData("gyro heading", angles.firstAngle);
                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
                telemetry.addData("if statement", degrees - currentOffset < 10);
                telemetry.addData("degrees + currentOffset", degrees + currentOffset);
                telemetry.update();
                idle();
            }
        }
        else
            while (opModeIsActive() && angles.firstAngle < targetLength)
            {
                telemetry.addData("gyro heading", angles.firstAngle);
                telemetry.update();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentOffset = angles.firstAngle;
                if(currentOffset >= 170) {currentOffset = -currentOffset + 360;}

//                if(degrees - currentOffset < 10){
//                    robot.frontLeft.setPower(leftPower/2*(degrees + currentOffset));
//                    robot.frontRight.setPower(rightPower/2*(degrees + currentOffset));
//                    robot.backLeft.setPower(leftPower/2*(degrees + currentOffset));
//                    robot.backRight.setPower(rightPower/2*(degrees + currentOffset));
//                }

                telemetry.addData("gyro heading", angles.firstAngle);
                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
                telemetry.update();
                idle();
            }

        // turn the motors off.a
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        // Reset gyro heading to zero on new direction we are now pointing.
        offset = angles.firstAngle;
        if(offset >= 170) {offset = -offset + 360;}
    }

    void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "Gyro calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();

        }
        telemetry.addData("Mode", "Gyro calibration done");
        telemetry.update();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
//    private double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle-0.37;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else if(movingForward)
            correction = -angle - offset;        // reverse sign of angle for correction.
        else
            correction = angle + offset;
        correction = correction * gain;

        return correction;
    }


    public void turn (HyperBot robot, double speed, double degrees, int direction, double timeoutS) {
        int frontLeftTarget = 0;
        int frontRightTarget = 0;
        int backLeftTarget = 0;
        int backRightTarget = 0;

        if (direction == TURNLEFT) {
            // Determine new target position, and pass to motor controller
            frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            backRightTarget = robot.backRight.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
        } else if (direction == TURNRIGHT) {
            // Determine new target position, and pass to motor controller
            frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            backRightTarget = robot.backRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
        }


        robot.frontLeft.setTargetPosition(frontLeftTarget);
        robot.frontRight.setTargetPosition(frontRightTarget);
        robot.backLeft.setTargetPosition(backLeftTarget);
        robot.backRight.setTargetPosition(backRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontLeft.setPower(Math.abs(speed));
        robot.frontRight.setPower(Math.abs(speed));
        robot.backLeft.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            if (minDistance > 0) {
//                while (opModeIsActive() &&
//                        (runtime.seconds() < timeoutS) &&
//                        robot.distanceSensor.getDistance(DistanceUnit.INCH) > minDistance) {
//                    sleep(50);
//                }
//            } else {
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
            sleep(50);
        }
//        }

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

    public void resetGyro() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        offset = angles.firstAngle;
        if(offset >= 170) {offset = -offset + 360;}
    }

    //start       startingPos     PIDStart targetPos
    //--|-------------|--------------|-------|
    //  0             10             18     20
    //          left/rightStart
    //                |----------------------|inches = 10
    //                               |-------|PIDDistance = 2
    //left/rightStart + inches - PIDDistance = PIDStart


    //  startingPos    PIDStart targetPos/start
    //-----|--------------|-------|
    //     -10           -2       0
    //left/rightStart
    //     |----------------------|inches = 10
    //                    |-------|PIDDistance = 2
    //left/rightStart + inches - PIDDistance = PIDStart

    //startingPos + inches = targetPos

    public double PID(int leftOdo, int rightOdo, int leftStart, int rightStart, double inches, double speed) {
        double startPos = (leftStart + rightStart) / 2;
        double pos = (leftOdo + rightOdo) / 2;
        startPos = startPos * ODOMETER_COUNTS_PER_INCH;
        pos = pos * ODOMETER_COUNTS_PER_INCH;
        double newSpeed = speed;
        if(pos > startPos + inches - PIDDistance) {
            newSpeed = ((startPos + inches) - pos) * (1/PIDDistance) * speed;
        }

//        if ((startPos + inches) - pos < 0){
//            newSpeed = 0;
//        }
//        System.out.println("newSpeed : " + newSpeed + "\nstartPos : " + startPos + "\nPosition" + pos + "\nspeed" + speed + "\nleft odometer : " + leftOdo * ODOMETER_COUNTS_PER_INCH + "\nright odometer : " + rightOdo * ODOMETER_COUNTS_PER_INCH + "\nPIDistance : " + PIDDistance);
//        System.out.println("\nconvert start : " + startPos);
//        System.out.println("\nadd inches : " + String.valueOf(startPos + inches));
//        System.out.println("\nconvert pos : " + pos);
//        System.out.println("\n(1/PIDDistance) : " + 1/PIDDistance);
//        System.out.println("\n(1/PIDDistance) * speed : " + (1/PIDDistance) * speed);


        return newSpeed;
    }

    public void lower(HyperBot robot) {
        robot.rampServo.setPosition(1);
    }
    public void drop(HyperBot robot) {
        robot.clawServo.setPosition(1);
    }

    public void close(HyperBot robot) {
        robot.clawServo.setPosition(0);
        robot.rampServo.setPosition(0);
    }

    public void spin(HyperBot robot, double rotations, double timeoutS) {
        //start spinning
        int sTarget = robot.frontLeft.getCurrentPosition() + (int) (rotations * COUNTS_PER_INCH);
        robot.sucker.setTargetPosition(sTarget);
        robot.sucker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.sucker.setPower(-0.7);

        //wait until we reach the target
        while (opModeIsActive() && runtime.seconds() < timeoutS && robot.sucker.isBusy()) {
            sleep(50);
        }

        //stop motion
        robot.sucker.setPower(0);
        robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinUp(HyperBot robot, double rotations, double timeoutS) {
        //start spinning
        runtime.reset();
        robot.sucker.setPower(-0.7);

        //wait until we reach the target
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            sleep(50);
        }

        //stop motion
        robot.sucker.setPower(0);
//        robot.sucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    int target = 0;

    public void armup1(HyperBot robot) {
        target = robot.armMotorRight.getCurrentPosition() + 1600;
        robot.armMotorRight.setTargetPosition(target);
        robot.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorRight.setPower(-1);
        while (robot.armMotorRight.isBusy()) {
            if (robot.armMotorRight.getCurrentPosition() >= target) {
                robot.armMotorRight.setPower(0);
                robot.armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }

    public void armup2(HyperBot robot) {
        target = robot.armMotorRight.getCurrentPosition() + 2100;
        robot.armMotorRight.setTargetPosition(target);
        robot.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorRight.setPower(-1);
        while (robot.armMotorRight.isBusy()) {
            if (robot.armMotorRight.getCurrentPosition() >= target) {
                robot.armMotorRight.setPower(0);
                robot.armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }

    public void armup3(HyperBot robot) {
        target = robot.armMotorRight.getCurrentPosition() + 2350;
        robot.armMotorRight.setTargetPosition(target);
        robot.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorRight.setPower(-1);
        while (robot.armMotorRight.isBusy()) {
            if (robot.armMotorRight.getCurrentPosition() >= target) {
                robot.armMotorRight.setPower(0);
                robot.armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }


    public void armdown (HyperBot robot, int distance) {
        target = robot.armMotorRight.getCurrentPosition() - distance;
        robot.armMotorRight.setTargetPosition(target);
        robot.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorRight.setPower(1);
        while(robot.armMotorRight.isBusy()) {
            if(robot.armMotorRight.getCurrentPosition()<=target) {
                robot.armMotorRight.setPower(0);
                robot.armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }


    public void spinCarouselTime(HyperBot robot, double timeoutS) {
        runtime.reset();
        robot.spinner.setPower(-0.03);
        sleep(5000);
        robot.spinner.setPower(0);
    }

    public void spinCarousel(HyperBot robot, double timeoutS) {
        target = robot.spinner.getCurrentPosition() - 2000;
        robot.spinner.setTargetPosition(target);
        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.spinner.setPower(0.05);
        while (robot.spinner.isBusy() && runtime.seconds() < timeoutS) {
            if (robot.spinner.getCurrentPosition() <= target) {
                robot.spinner.setPower(0);
                robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
        robot.spinner.setPower(0);

        target = robot.spinner.getCurrentPosition() - 2000;
        robot.spinner.setTargetPosition(target);
        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinner.setPower(0.05);
        while (robot.spinner.isBusy() && runtime.seconds() < timeoutS) {
            if (robot.spinner.getCurrentPosition() <= target) {
                robot.spinner.setPower(0);
                robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
        robot.spinner.setPower(0);
        robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lowerOdo(HyperBot robot) {
        robot.leftServo.setPosition(1);
        robot.rightServo.setPosition(0);
        robot.backServo.setPosition(1);
    }
    public void raiseOdo(HyperBot robot) {
        robot.leftServo.setPosition(0);
        robot.rightServo.setPosition(1);
        robot.backServo.setPosition(0);
    }


}