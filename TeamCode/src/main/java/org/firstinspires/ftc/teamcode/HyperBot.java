package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.teamcode.util.Encoder;

public class HyperBot {
    /* Public OpMode members. */
    //null = nothing, not 0
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft    = null;
    public DcMotor  backRight   = null;
    public DcMotor  armMotor    = null;
    public DcMotor  spinner     = null;
    public DcMotor  lEncoder    = null;
    public DcMotor  rEncoder    = null;
    public DcMotor  bEncoder    = null;
    public DcMotor  sucker      = null;
    public int lEncoderDirection = 1;
    public int rEncoderDirection = 1;
    public int bEncoderDirection = -1;
    //    public DcMotor  conveyor   = null;
//    public DcMotor  linearDrive = null;
//    public DcMotor    linearServo = null;
    public Servo    leftServo   = null;
    public Servo    backServo   = null;
    public Servo    rightServo   = null;
    public Servo    clawServo   = null;

    double left;
    double right;
    double back;
    double left2 = left;
    double right2 = right;
    double back2 = back;
    double n1 = 0;
    double n2 = 0;
    double n3 = 0;
    double ldistance = 0;
    double rdistance = 0;
    double bdistance = 0;
    double distancex = 0;
    double distancey = 0;
    double heading = 0;
    double centerside = 11.75;
    double centerback = 5.75;
    double diffx = 0;
    double diffy = 0;
    double diffhead = 0;
//
//    //intake
////    public RevRobotics40HdHexMotor intakeArmLeft = null;
////    public RevRobotics40HdHexMotor intakeArmRight = null;
////    public DcMotor intakeLeft        = null;
////    public DcMotor intakeRight       = null;
////    public DcMotor intakeArm         = null;
//    public DcMotor shooterLeft         = null;
//    public DcMotor shooterRight         = null;
////    public Servo   intakePusher   = null;
//    public Servo   clawArm        = null;


//    public ColorSensor colorSensor = null;
//    public DistanceSensor distanceSensor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HyperBot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //hardwaremap = robot config

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        bEncoder = hwMap.get(DcMotorEx.class, "spinner");
        lEncoder = hwMap.get(DcMotorEx.class, "sucker");
        rEncoder = hwMap.get(DcMotorEx.class, "rEncoder");
        rEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        sucker = hwMap.get(DcMotor.class, "sucker");
        spinner = hwMap.get(DcMotor.class, "spinner");


        // Set all motors to run without encoder by default
        //encoder = fll degrees
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sucker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
//        armMotor.setDirection(DcMotor.Direction.FORWARD);

//        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
//        armMotor.setPower(0);
//        conveyor.setPower(0);

        // Intake motors
//        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
//        intakeRight = hwMap.get(DcMotor.class, "intakeRight");
//        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
//        intakeRight.setDirection(DcMotor.Direction.FORWARD);
//        intakeLeft.setPower(0);
//        intakeRight.setPower(0);
//        intakeArm = hwMap.get(DcMotor.class, "intakeArm");
//        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeArm.setPower(0);
//
//        // shooter
//        shooterLeft = hwMap.get(DcMotor.class, "shooterLeft");
//        shooterRight = hwMap.get(DcMotor.class, "shooterRight");
//        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
//        shooterRight.setDirection(DcMotor.Direction.FORWARD);
//        shooterLeft.setPower(0);
//        shooterRight.setPower(0);
//
//        // Define and initialize servos
//      //  linearServo  = hwMap.crservo.get("linearServo");
////        linearDrive = hwMap.get(DcMotor.class, "linearDrive");
        leftServo  = hwMap.get(Servo.class, "leftServo");
        backServo  = hwMap.get(Servo.class, "backServo");
        rightServo  = hwMap.get(Servo.class, "rightServo");
        clawServo  = hwMap.get(Servo.class, "clawServo");
        clawServo.setPosition(1);
////        intakePusher  = hwMap.get(Servo.class, "intakePusher");
////        intakePusher.setPosition(1);
//        clawArm  = hwMap.get(Servo.class, "clawArm");



//        public static int getCurrentMotorPos(DcMotor motor) {
//            return motor.getCurrentPosition();
//        }

//        colorSensor = ahwMap.get(ColorSensor.class, "colorSensor");
//        distanceSensor = ahwMap.get(DistanceSensor.class, "distanceSensor");

//        getCurrentPosition();
        left = lEncoder.getCurrentPosition();
        right = rEncoder.getCurrentPosition();
        back = bEncoder.getCurrentPosition();
        left2 = left;
         right2 = right;
         back2 = back;
        return;


    }

    public void getCurrentPosition() {
        TelemetryPacket packet = new TelemetryPacket();

//


            left = lEncoder.getCurrentPosition();
            right = rEncoder.getCurrentPosition();
            back = bEncoder.getCurrentPosition();
            n1 = left - left2;
            n2 = right - right2;
            n3 = back - back2;
            ldistance = (n1/8192)*2*Math.PI*0.6889763779527559;
            rdistance = (n2/8192)*2*Math.PI*0.6889763779527559;
            bdistance = (n3/8192)*2*Math.PI*0.6889763779527559;
            diffx = (ldistance+rdistance)/2;
            diffy = (bdistance - centerback*(rdistance-ldistance)/centerside);
            diffhead = (rdistance-ldistance)/centerside;
            distancex = distancex + diffx * Math.cos(heading) - diffy * Math.sin(heading);
            distancey = distancey + diffy*Math.cos(heading)+ diffx;
            heading = heading + diffhead;
            left2 = left;
            right2 = right;
            back2 = back;

        packet.put("X position:  ", distancex);
        packet.put("Y position:  ", distancey);
        packet.put("Heading:  ", heading);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public void lowerOdo(){
        leftServo.setPosition(1);
        rightServo.setPosition(0);
        backServo.setPosition(1);
    }
    public void raiseOdo() {
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        backServo.setPosition(0);
    }


}

