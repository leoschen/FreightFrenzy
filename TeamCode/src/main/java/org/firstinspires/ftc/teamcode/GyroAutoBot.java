//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Autonomous(name="GyroAutoBot", group="HGT")
//@Disabled
//public class GyroAutoBot extends LinearOpMode {
//    HyperBot           robot = new HyperBot();
//    ElapsedTime runtime = new ElapsedTime();
//
//    static final double     COUNTS_PER_ROTATION_HD_HEX_MOTOR = 1120;
//    static final double     WHEEL_DIAMETER_INCHES   = 3.94 ;     // 75mm
//    static final double     COUNTS_PER_INCH = COUNTS_PER_ROTATION_HD_HEX_MOTOR / (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    static final int FORWARD = 1;
//    static final int BACK = 2;
//    static final int RIGHT = 3;
//    static final int LEFT = 4;
//    static final int TURNLEFT = 5;
//    static final int TURNRIGHT = 6;
//
//    boolean movingForward = true;
//    double turn;
//    double offset = 0;
//
//    BNO055IMU               imu;
//    Orientation             lastAngles = new Orientation();
//    double                  globalAngle, correction;
//
//    @Override
//    public void runOpMode() {
//        // do nothing here. All classes that extends AutoBot need to override this method
//    }
//
//    public void setup() {
//        robot.init(hardwareMap);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        initGyro();
//
//    }
//
//    public void move(HyperBot robot, double speed, double inches, int timeoutS, int direction) {
//        moveInternal(robot, speed, inches, timeoutS, direction, -1);
//    }
//
//    public void moveWithDistanceSensor(HyperBot robot, double speed, double inches, int timeoutS, int direction, double minDistance) {
//        moveInternal(robot, speed, inches, timeoutS, direction, minDistance);
//    }
//
//    private void moveInternal(HyperBot robot, double speed, double inches, int timeoutS, int direction, double minDistance) {
//        int frontLeftTarget = 0;
//        int frontRightTarget = 0;
//        int backLeftTarget = 0;
//        int backRightTarget = 0;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//            if (direction == BACK) {
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                movingForward = false;
//            } else if (direction == FORWARD) {
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                movingForward = true;
//            } else if (direction == LEFT) {
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                backRightTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            } else if (direction == RIGHT) {
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//                frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//                backRightTarget = robot.backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
//            }
//
//            robot.frontLeft.setTargetPosition(frontLeftTarget);
//            robot.frontRight.setTargetPosition(frontRightTarget);
//            robot.backLeft.setTargetPosition(backLeftTarget);
//            robot.backRight.setTargetPosition(backRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            do {
//                // get the direction to correct
//                correction = checkDirection();
//
//                telemetry.addData("1 imu heading:", lastAngles.firstAngle);
//                telemetry.addData("2 global heading:", globalAngle);
//                telemetry.addData("3 correction:", correction);
//                telemetry.update();
//
//                //move it in the right direction and add the correction adjustment to the speed
//                if (direction == RIGHT) {
//                    robot.frontLeft.setPower(Math.abs(speed) - (correction * 0.6));//0.6
//                    robot.frontRight.setPower(Math.abs(speed) + (correction * 0.6));//0.6
//                    robot.backLeft.setPower(Math.abs(speed) - (correction * 0.6));//0.6
//                    robot.backRight.setPower(Math.abs(speed) + (correction * 0.6));//0.6
//                } else {
//                    robot.frontLeft.setPower(Math.abs(speed) + (correction * 0.6));//0.6
//                    robot.frontRight.setPower(Math.abs(speed) - (correction * 0.6));//0.6
//                    robot.backLeft.setPower(Math.abs(speed) + (correction * 0.6));//0.6
//                    robot.backRight.setPower(Math.abs(speed) - (correction * 0.6));//0.6
//                }
//
//                sleep(6);
//
//            } while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()));
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            double nOffset = angles.firstAngle;
//            if(nOffset >= 170) {nOffset = -nOffset + 360;}
//
//            if (nOffset == offset) {
//
//            }else if (nOffset - offset > 0) {
//                robot.frontLeft.setPower(0.3);
//                robot.frontRight.setPower(-0.3);
//                robot.backLeft.setPower(0.3);
//                robot.backRight.setPower(-0.3);
//                while(nOffset - offset > 0) {
//                    sleep(6);
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    nOffset = angles.firstAngle;
//                    if(nOffset >= 170) {nOffset = -nOffset + 360;}
//                    telemetry.addData("right:", angles.firstAngle);
//                    telemetry.update();
//                    if(nOffset - offset < 0){
//                        break;
//                    }
//                }
//            } else {
//                robot.frontLeft.setPower(-0.3);
//                robot.frontRight.setPower(0.3);
//                robot.backLeft.setPower(-0.3);
//                robot.backRight.setPower(0.3);
//                while(nOffset - offset < 0) {
//                    sleep(6);
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    nOffset = angles.firstAngle;
//                    if(nOffset >= 170) {nOffset = -nOffset + 360;}
//                    telemetry.addData("left:", angles.firstAngle);
//                    telemetry.update();
//                    if(nOffset - offset > 0){
//                        break;
//                    }
//                }
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.backRight.setPower(0);
//
//
//
//        }
//
//
//    }
//
//    public void gyroTurn (HyperBot robot, double power, double degrees, int direction){
//
//        double  leftPower, rightPower;
//        double  targetAngle;
//        double  targetLength;
//        double  currentOffset;
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        // set offset/the new angle to go on
//
//        offset = angles.firstAngle;
//        if(offset >= 170) {offset = -offset + 360;}
//
//        // Gyro returns 0->359 when rotating counter clockwise (left) and 359->0 when rotating
//        // clockwise (right).
//
//        if (direction == RIGHT) {   // turn right.
//            leftPower = power;
//            rightPower = -power;
//            targetAngle = 360 - degrees + offset;    // degrees is - for right turn.
//            targetLength = offset - degrees;
//        } else if (direction == LEFT) {   // turn left.
//            leftPower = -power;
//            rightPower = power;
//            targetAngle = degrees + offset;
//            targetLength = offset + degrees;
//        } else return;
//
//        // set power to rotate.
//        robot.frontLeft.setPower(leftPower);
//        robot.frontRight.setPower(rightPower);
//        robot.backLeft.setPower(leftPower);
//        robot.backRight.setPower(rightPower);
//
//        // rotate until turn is completed.
//        if (direction == RIGHT)
//        {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && angles.firstAngle == 0) {
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                currentOffset = angles.firstAngle;
//                if(currentOffset >= 170) {currentOffset = -currentOffset + 360;}
//
//                if(targetLength - currentOffset < -10){
//                    robot.frontLeft.setPower(leftPower/2*(targetLength - currentOffset));
//                    robot.frontRight.setPower(rightPower/2*(targetLength - currentOffset));
//                    robot.backLeft.setPower(leftPower/2*(targetLength - currentOffset));
//                    robot.backRight.setPower(rightPower/2*(targetLength - currentOffset));
//                }
//
//                telemetry.addData("gyro heading", angles.firstAngle);
//                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
//                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
//                telemetry.update();
//                idle();
//            }
//
//            while (opModeIsActive() && angles.firstAngle > targetLength) {
//
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                currentOffset = angles.firstAngle;
//                if(currentOffset >=170) {currentOffset = -currentOffset + 360;}
//
////                if(degrees + currentOffset < 10){
////                    robot.frontLeft.setPower(leftPower/2*(degrees + currentOffset));
////                    robot.frontRight.setPower(rightPower/2*(degrees + currentOffset));
////                    robot.backLeft.setPower(leftPower/2*(degrees + currentOffset));
////                    robot.backRight.setPower(rightPower/2*(degrees + currentOffset));
////                }
//
//                telemetry.addData("gyro heading", angles.firstAngle);
//                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
//                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
//                telemetry.addData("if statement", degrees - currentOffset < 10);
//                telemetry.addData("degrees + currentOffset", degrees + currentOffset);
//                telemetry.update();
//                idle();
//            }
//        }
//        else
//            while (opModeIsActive() && angles.firstAngle < targetLength)
//            {
//                telemetry.addData("gyro heading", angles.firstAngle);
//                telemetry.update();
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                currentOffset = angles.firstAngle;
//                if(currentOffset >= 170) {currentOffset = -currentOffset + 360;}
//
////                if(degrees - currentOffset < 10){
////                    robot.frontLeft.setPower(leftPower/2*(degrees + currentOffset));
////                    robot.frontRight.setPower(rightPower/2*(degrees + currentOffset));
////                    robot.backLeft.setPower(leftPower/2*(degrees + currentOffset));
////                    robot.backRight.setPower(rightPower/2*(degrees + currentOffset));
////                }
//
//                telemetry.addData("gyro heading", angles.firstAngle);
//                telemetry.addData("power of frontLeft", (leftPower/2*(targetLength - currentOffset)));
//                telemetry.addData("real power of frontLeft", robot.frontLeft.getPower());
//                telemetry.update();
//                idle();
//            }
//
//        // turn the motors off.a
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//
//        // Reset gyro heading to zero on new direction we are now pointing.
//        offset = angles.firstAngle;
//        if(offset >= 170) {offset = -offset + 360;}
//    }
//
//
//
//    public void openClaw (HyperBot robot) {
//        robot.clawServo.setPosition(1);
//    }
//
//    public void closeClaw (HyperBot robot) {
//        robot.clawServo.setPosition(0);
//    }
//
//    void initGyro() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//
//        telemetry.addData("Mode", "Gyro calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//
//        }
//        telemetry.addData("Mode", "Gyro calibration done");
//        telemetry.update();
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     * @return Angle in degrees. + = left, - = right.
//     */
////    private double getAngle()
////    {
////        // We experimentally determined the Z axis is the axis we want to use for heading angle.
////        // We have to process the angle because the imu works in euler angles so the Z axis is
////        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
////        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
////
////        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////
////        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
////
////        if (deltaAngle < -180)
////            deltaAngle += 360;
////        else if (deltaAngle > 180)
////            deltaAngle -= 360;
////
////        globalAngle += deltaAngle;
////
////        lastAngles = angles;
////
////        return globalAngle;
////    }
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
//        return globalAngle-0.37;
//    }
//
//    /**
//     * See if we are moving in a straight line and if not return a power correction value.
//     * @return Power adjustment, + is adjust left - is adjust right.
//     */
//    private double checkDirection()
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else if(movingForward)
//            correction = -angle - offset;        // reverse sign of angle for correction.
//        else
//            correction = angle + offset;
//        correction = correction * gain;
//
//        return correction;
//    }
//
//
//
//    int target = 0;
////    public void armup (HyperBot robot) {
////        target = robot.armMotor.getCurrentPosition()+1600;//1600
////        robot.armMotor.setTargetPosition(target);
////        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.armMotor.setPower(-1);
////        while(robot.armMotor.isBusy()) {
////            if(robot.armMotor.getCurrentPosition()==target) {
////                robot.armMotor.setPower(0);
////                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                break;
////            }
////        }
////    }
////
////    public void armdown (HyperBot robot) {
////        target = robot.armMotor.getCurrentPosition()-1580;//-1580
////        robot.armMotor.setTargetPosition(target);
////        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.armMotor.setPower(1);
////        while(robot.armMotor.isBusy()) {
////            if(robot.armMotor.getCurrentPosition()==target) {
////                robot.armMotor.setPower(0);
////                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                break;
////            }
////        }
////    }
//
//    public void turn (HyperBot robot, double speed, double degrees, int direction, double timeoutS) {
//        int frontLeftTarget = 0;
//        int frontRightTarget = 0;
//        int backLeftTarget = 0;
//        int backRightTarget = 0;
//
//        if (direction == TURNLEFT) {
//            // Determine new target position, and pass to motor controller
//            frontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
//            frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
//            backLeftTarget = robot.backLeft.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
//            backRightTarget = robot.backRight.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
//        } else if (direction == TURNRIGHT) {
//            // Determine new target position, and pass to motor controller
//            frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
//            frontRightTarget = robot.frontRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
//            backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
//            backRightTarget = robot.backRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
//        }
//
//
//        robot.frontLeft.setTargetPosition(frontLeftTarget);
//        robot.frontRight.setTargetPosition(frontRightTarget);
//        robot.backLeft.setTargetPosition(backLeftTarget);
//        robot.backRight.setTargetPosition(backRightTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        robot.frontLeft.setPower(Math.abs(speed));
//        robot.frontRight.setPower(Math.abs(speed));
//        robot.backLeft.setPower(Math.abs(speed));
//        robot.backRight.setPower(Math.abs(speed));
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
////            if (minDistance > 0) {
////                while (opModeIsActive() &&
////                        (runtime.seconds() < timeoutS) &&
////                        robot.distanceSensor.getDistance(DistanceUnit.INCH) > minDistance) {
////                    sleep(50);
////                }
////            } else {
//        while (opModeIsActive() &&
//                (runtime.seconds() < timeoutS) &&
//                (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
//            sleep(50);
//        }
////        }
//
//        // Stop all motion;
//
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void resetGyro() {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        offset = angles.firstAngle;
//        if(offset >= 170) {offset = -offset + 360;}
//    }
//
//}
