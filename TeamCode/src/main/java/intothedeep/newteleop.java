package intothedeep;

import static java.lang.Math.abs;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.constants.PinpointConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import intothedeep.Arm.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import intothedeep.Arm;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;


@Config
@TeleOp
public class newteleop extends LinearOpMode {
    public static final double[] speed = {0.25, 0.65, 1};

    double subinout = 0.41;
    double inandout = 0.28; //0.2

    private Timer ascenttimer;
    DcMotorEx leftslide, rightslide;
    DcMotorEx leftpivot, rightpivot;
    DcMotorEx FL, FR, BL, BR;
    Servo claw, clawswivel, singleclawrotate, leftclawrotate, rightclawrotate;

    public static double clawswivelcenter = 0.181, clawswivelleft = 0.6, clawswivelright = 0;

    final static double p = 0.0055, i = 0, d = 0.0001;
    double f = 0.05;
    int target = 0;
    private int bumperstage = 0;
    private int lefttriggerstage = 0;
    private int righttriggerstage = 0;
    private int slideslengthstage = 1;
    private boolean rightbumperpressed = false;
    private boolean righttriggerpressed = false;
    private boolean lefttriggerpressed = false;
    private boolean buttonApressed = false;
    private boolean buttonBpressed = false;
    private int ascentstage = 0;
    int caseonepos = 150;

    boolean driverleftbumper = false;
    boolean driverrightbumper = false;

    void setPIDFCoefficients(DcMotorEx.RunMode mode, PIDFCoefficients pidf) {
        leftslide.setPIDFCoefficients(mode, pidf);
        rightslide.setPIDFCoefficients(mode, pidf);
    }

    public void slides(int slidesposition) {
        leftslide.setTargetPosition(slidesposition);
        rightslide.setTargetPosition(slidesposition);
        leftslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftslide.setPower(1);
        rightslide.setPower(1);
    }

    public void slidespositio(int slidespos) {
        leftslide.setTargetPosition(slidespos);
        rightslide.setTargetPosition(slidespos);
        leftslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftslide.setPower(1);
        rightslide.setPower(1);
    }

    public void largeclawrotate(double rotate) {
        leftclawrotate.setPosition(rotate);
        rightclawrotate.setPosition(rotate);
    }
    public void samplebasketrelease() {
        largeclawrotate(0);
        clawswivel.setPosition(0.22); //maker sure center
//        singleclawrotate.setPosition(0.6); //figure out what to set this
    }
    public void submersibleinandout() {
        largeclawrotate(inandout);
//        singleclawrotate.setPosition(0.15); //figure out what to set this
    }
    public void samplepickuppos() {
        largeclawrotate(0.01);
//        singleclawrotate.setPosition(0.17); //figure out what to set this
    }
    public void resetpos() {
        largeclawrotate(0.65);
    }
    boolean clawpressed = false;
    public void pivotposition(int pivottarget, double pivotspeed) {
        leftpivot.setTargetPosition(pivottarget);
        rightpivot.setTargetPosition(pivottarget);
        leftpivot.setPower(pivotspeed);
        rightpivot.setPower(pivotspeed);
        leftpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    double rx;
    double x;
    double y;
    double denominator;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    @Override
    public void runOpMode() throws InterruptedException {
        //SLIDES:
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);
        rightslide.setDirection(DcMotorEx.Direction.REVERSE);
//        Arm leftslide = new Arm( "leftslide", p, i, d, f);
//        Arm rightslide = new Arm( "rightslide", p, i, d, f);
//        Arm slides = new Arm( "arm_motor", p, i, d, f);
//        Arm slides = new Arm(hardwareMap, "arm_motor", p, i, d, f);
        Arm leftslides = new Arm(hardwareMap, "leftslide", p, i, d, f);

        leftslide.setPositionPIDFCoefficients(0.0055);
        rightslide.setPositionPIDFCoefficients(0.0055);
        leftslide.setVelocityPIDFCoefficients(0.0055, 0, 0.0001, 0.1);
        rightslide.setVelocityPIDFCoefficients(0.0055, 0, 0.0001, 0.1);

        //PIVOT
        leftpivot = hardwareMap.get(DcMotorEx.class, "leftpivot");
        rightpivot = hardwareMap.get(DcMotorEx.class, "rightpivot");
        leftpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //maybe change later for specimen
        rightpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftpivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightpivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightpivot.setDirection(DcMotorEx.Direction.REVERSE);


        //DRIVETRAIN
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);
//        BL.setDirection(DcMotorEx.Direction.REVERSE);
//        FL.setDirection(DcMotorEx.Direction.REVERSE);
//        FR.setDirection(DcMotorEx.Direction.REVERSE);

        //trying out field centric drivetrain
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);


        byte speedx = 2;

        //SERVOS
        leftclawrotate = hardwareMap.get(Servo.class, "leftend");
        rightclawrotate = hardwareMap.get(Servo.class, "rightend");
        //range  for big rotate is 0.01 - 0.33. 0.01 is towards front
        leftclawrotate.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw"); //0.2 = open. 0.46 = close.
        clawswivel = hardwareMap.get(Servo.class, "wristXTilt"); //0 = left. 0.19 = center. 0.39 = right.
        singleclawrotate = hardwareMap.get(Servo.class, "wristSwerve"); //0 = towards front. 0.7 = towards back.

        waitForStart();
        double prevArmPower = 0.0;


        if (isStopRequested()) return;

        boolean clawtoggle = true;

        while (opModeIsActive()) {

            /** OPERATOR STUFF: */
            //opening and closing claw. if this doestn work then use previous gamepads on gm0. TRUE = OPEN, FALSE = CLOSE
            if (gamepad1.left_bumper && !clawpressed) {
                clawpressed = true;
                if (clawtoggle) {
                    clawtoggle = false;
                } else if (!clawtoggle) {
                    clawtoggle = true;
                }
            }
            if (!gamepad1.left_bumper) {
                clawpressed = false;
            }
            if (clawtoggle) {
                claw.setPosition(0.52); //open
            } else if (!clawtoggle) {
                claw.setPosition(0.18); //close
            }

            //FIGURE OUT NEW VALUES
            if ((gamepad1.left_stick_x < -0.5) && (clawswivel.getPosition() < 0.7)) {
                clawswivel.setPosition(clawswivel.getPosition() + 0.015); //turns claw to the right
            }
            if ((gamepad1.left_stick_x > 0.5) && clawswivel.getPosition() > 0) {
                clawswivel.setPosition(clawswivel.getPosition() - 0.015); //turns claw to the left
            }
            if (gamepad1.left_stick_y > 0.5 || gamepad1.left_stick_y < -0.5 || gamepad1.left_stick_button) {
                clawswivel.setPosition(0.22); //resets to 0 degree angle
            }


            //RIGHT BUMPER SAMPLE PICKUP AND SCORE
            if (gamepad1.right_bumper && !rightbumperpressed) {
                rightbumperpressed = true;
                if (bumperstage > 3.9 && bumperstage < 6.9) {
//                    rightbumperpressed  = true; //or false?
                    bumperstage = 7;
                } else if (bumperstage > 6.9) {
//                    rightbumperpressed = true; //or true?
                    bumperstage = 1;
                } else {
                    bumperstage++;
                }
            } else if (!gamepad1.right_bumper) {
                rightbumperpressed = false;
            }

            //available buttons - y, dpad_right, left_trigger, right_trigger, right_stick
            //to collect pixel leaning on wall and cone on its side
            if (gamepad1.y) {
                caseonepos = 350;
                bumperstage = 20; //ends bumperstage loop. next time right bumper is pressed, bumper stage loop will resume at case 1
                largeclawrotate(0.8);
//                singleclawrotate.setPosition(0.45);
            }


            if (gamepad1.b && !buttonBpressed) {
                buttonBpressed = true;
                if (slideslengthstage > 2.9) {
                    slideslengthstage = 1;
                } else {
                    slideslengthstage++;
                }
            } else if (!gamepad1.b) {
                buttonBpressed = false;
            }
            switch (slideslengthstage) {
                case 1:
                    caseonepos = 325;
                    break;
                case 2:
                    caseonepos = 900;
                    break;
                case 3:
                    caseonepos = 1450; //if robot is out of size decrease this
                    break;
            }

            switch (bumperstage) {
                case 1: //slides and servos position for observation zone entry
                    //set claw mech positions in order to be able to move in and out of sub but as low as possible
                    //   leftslides.setTarget(1480);
                    submersibleinandout();
//                    pivotposition(5, 0.35);
//                    slides(caseonepos);
                    leftslides.setTarget(caseonepos);
//                    slidespositio(caseonepos);
                    break;

                case 2: //slides and servos position once in observation zone
                    //once sample is located and have top of claw go down enough to touch sample and close claw becomes part of stage
                    samplepickuppos();
                    leftslides.setTarget(caseonepos);
                    break;

                case 3: //close claw, reset claw mech position, retract slides (not fully). (claw will be closed seperately to make sure that sample is picked up)
                    clawtoggle = false;
                    leftslides.setTarget(caseonepos);
                    break;

                case 4:
                    resetpos();
//                    slides(50);
                    leftslides.setTarget(50);
                    break;

                case 7: //make slides vertical, fix claw mech position for sample release into basket, extend slides FULLY. make sure servos move simultaneously with other mechs
//                    pivotposition(268, 1);
                    largeclawrotate(0.85);
                    leftslides.setTarget(500);
                    break;

//                case 8: //automatically happens
//                    samplebasketrelease();
//                    largeclawrotate(0);
//                    clawswivel.setPosition(0.22);
////                    slides(2374);
////                    leftslides.setTarget(2380);
////                    slidespositio(2374);
////                    pivotposition(270, 0.7);
////                    leftpivot.setTargetPosition(270);
////                    rightpivot.setTargetPosition(270);
////                    leftpivot.setPower(0.7);
////                    rightpivot.setPower(0.7);
////                    leftpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    rightpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    largeclawrotate(0);
//                    break;
//
//                case 9: //open claw, retract slides, horizontal pivot
//                    clawtoggle = true; //open
////                    pivotposition(270, 0.5);
//                    break;
//
//                case 10:
//                    // slides(20);
//                    leftslides.setTarget(40); // retract
//                    clawswivel.setPosition(0.22);
//                    caseonepos = 200;
//                    break;
//
//                case 11: //automatically happens
////                    pivotposition(10, 1); //horizontal
//                    largeclawrotate(0.45);
//                    caseonepos = 210;
//                    break;
            }
            telemetry.addData("Sample Stage:", bumperstage);
            if (bumperstage == 11 && (leftpivot.getCurrentPosition() < 20)) {
                bumperstage = 1;
            }
//            if ((bumperstage == 7 || bumperstage == 8 || bumperstage == 9 || bumperstage == 10) && leftpivot.getCurrentPosition() < 255) {
////                pivotposition(270, 1);
//                //if it tries to fight against itself (not like how it can still fall now) then delete the following 6 lines
//                leftpivot.setTargetPosition(270);
//                rightpivot.setTargetPosition(270);
//                leftpivot.setPower(1);
//                rightpivot.setPower(1);
//                leftpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

            if (bumperstage == 10 && (leftslide.getCurrentPosition() < 80)) {
                bumperstage = 11;
            }

            if (gamepad1.right_stick_button) {
                bumperstage = 1;
                clawtoggle = true; //open
            }

            //ASCENT
            if (gamepad1.x && !buttonApressed) {
                buttonApressed = true;
                ascentstage++;
                if (ascentstage > 6) { //figure out exact
                    ascentstage = 1;
                }
            } else if (!gamepad1.x) {
                buttonApressed = false;
            }
            switch (ascentstage) { //maybe use pidf for pivot? AND WILL EITHER NEED MORE STAGES OR SEQUENTIAL ACTIONS
                case 1: //make slides vertical, extend to position ~750 - get into position. make sure claw mech is out of way but not hanging out on either side
                    pivotposition(270, 1);
                    break;
                case 2: //move pivot to position ~70. then retract slides to position ~250 and move ascent servos to position. sequential action? or make servos seperate stage
                    leftslides.setTarget(1700);
//                    slides(1500);
                    pivotposition(268, 1);
                    break;
                case 3: //extend slides fully and *then* move pivot to be able to get onto high bar
                    f = 0.1;
                    largeclawrotate(0);
                    pivotposition(155, 1);
                    largeclawrotate(0);
                    break;
                case 4: //retract slides so robot can hang off high bar then release ascent servos from low bar and retract slides rest of the way. then put ascent servos back and release slides
                    leftslides.setTarget(700);
                    leftpivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightpivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    leftslides.setTarget(150);
                    break;
                case 5:
                    break;
            }
            if ((leftpivot.getCurrentPosition() < 80) && (leftslide.getCurrentPosition() > 1850) && (bumperstage == 8 || bumperstage == 9 || bumperstage == 10)) {
                leftslides.setTarget(1400);
            }

            if (gamepad2.dpad_down) {
                leftclawrotate.setPosition(0);
            }
            if (gamepad2.y) {
                rightclawrotate.setPosition(0);
            }
            if (gamepad2.x) {
                singleclawrotate.setPosition(0.5);
            }
            if (gamepad2.dpad_up) {
                singleclawrotate.setPosition(0);
            }

            if (gamepad1.dpad_up) {
                inandout = inandout + 0.003;
            }
            if (gamepad1.dpad_down) {
                inandout = inandout - 0.003;
            }

            //driver can hold left or right bumper to decrease max speed and scale that onto the controller
            if (gamepad2.left_bumper) {
                speedx = 0;
            } else if (gamepad2.right_bumper) {
                speedx = 1;
            } else {
                speedx = 2;
            }

            rx = gamepad2.left_stick_y; // Remember, Y stick value is reversed
            x = -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            y = gamepad2.right_stick_x * 0.85;
            denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);

            frontLeftPower = speed[speedx] * ((y + x + rx) / denominator);
            backLeftPower = speed[speedx] * ((y - x + rx) / denominator);
            frontRightPower = speed[speedx] * ((y - x - rx) / denominator);
            backRightPower = speed[speedx] * ((y + x - rx) / denominator);

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            telemetry.update();

            if (bumperstage == 1 || bumperstage == 2 || bumperstage == 3 || bumperstage == 4 || bumperstage == 7 ||  bumperstage == 8 || bumperstage == 9 || bumperstage == 10 ||  ascentstage == 2 || ascentstage == 3 || ascentstage == 4 || ascentstage == 5 || ascentstage == 6 || ascentstage == 7) {
                leftslides.armTask();
            }

        }
    }
}