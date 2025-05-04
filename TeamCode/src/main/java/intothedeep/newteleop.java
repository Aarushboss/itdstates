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


@Config
@TeleOp
public class newteleop extends LinearOpMode {
    public static final double[] speed = {0.25, 0.65, 1};
    DcMotorEx leftslide, rightslide;
    DcMotorEx leftpivot, rightpivot;
    DcMotorEx FL, FR, BL, BR;
    Servo claw, clawswivel, singleclawrotate, leftclawrotate, rightclawrotate;

    public static double clawswivelcenter = 0.181, clawswivelleft = 0.6, clawswivelright = 0;

    final static double p = 0.0055, i = 0, d = 0.0001;
    final static double f = 0.1;
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
        largeclawrotate(0.25);
        clawswivel.setPosition(0.22); //maker sure center
        singleclawrotate.setPosition(0.24); //figure out what to set this
    }
    public void submersibleinandout() {
        largeclawrotate(0.28);
        singleclawrotate.setPosition(0); //figure out what to set this
    }
    public void samplepickuppos() {
        largeclawrotate(0.11);
        singleclawrotate.setPosition(0.05); //figure out what to set this
    }
    public void resetpos() {
        largeclawrotate(0.2);
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

            setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));


//            double armPower = -gamepad1.right_stick_y;
//            armPower = Math.abs(armPower) >= 0.25 ? armPower : 0.0;

//            if (gamepad1.right_stick_y > 0.25 || gamepad1.right_stick_y < -0.25) {
//                // Joystick action always interrupts and overrides button action.
//                leftslides.setPower(armPower);
////                prevArmPower = armPower;
//            } if (gamepad1.right_stick_y < 0.25 && gamepad1.right_stick_y > -0.25) {
//                leftslides.setPower(0);
//            }
//                else if (prevArmPower != 0.0) {
//                // Operator just let go the joystick. Stop the arm.
//                leftslides.setPower(0.0);
//                prevArmPower = 0.0;
//            }

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
                claw.setPosition(0.46); //open
            } else if (!clawtoggle) {
                claw.setPosition(0.1615); //close
            }
            //fully extending and retracting slides horizontally for sample release into observation zone for specimen cycles. and extending and retracting slides for submersible
            if (gamepad1.left_stick_y > 0.5) {
                //EXTEND SLIDES HORIZONTAL FULL (with pid)
            } else if (gamepad1.left_stick_y < -0.5) {
                clawtoggle = true;
                // RETRACT SLIDES HALFWAY (with pid). POSITION ~500
            }

            //CLAW SWIVEL:
//            if (gamepad1.right_stick_x > 0.5) { //50 is supposed to be ~halfway
//                clawswivel.setPosition(0.0); //60ish degree angle to right
//            } else if (gamepad1.right_stick_x < -0.5) {
//                clawswivel.setPosition(0.38); //60ish degree angle to left
//            }
//            // else if (gamepad1.right_stick_y < -0.5) {
//             //   clawswivel.setPosition(0.0); //90 degree angle
//           // }
//             else if (gamepad1.right_stick_y > 0.5 || gamepad1.right_stick_y < -0.5 // gamepad1.right_stick_button) {
//                clawswivel.setPosition(0.19); //resets to 0 degree angle
//            }
            //FIGURE OUT NEW VALUES
            if ((gamepad1.left_stick_x < -0.5) && (clawswivel.getPosition() < 0.7)) {
                clawswivel.setPosition(clawswivel.getPosition() + 0.013); //turns claw to the right
            }
            if ((gamepad1.left_stick_x > 0.5) && clawswivel.getPosition() > 0) {
                clawswivel.setPosition(clawswivel.getPosition() - 0.013); //turns claw to the left
            }
            if (gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3 || gamepad1.left_stick_button) {
                clawswivel.setPosition(0.22); //resets to 0 degree angle
            }
//            } if (gamepad1.left_stick_x > 0.3) {
//                clawswivel.setPosition(0);
//            } if (gamepad1.left_stick_x < -0.3) {
//                clawswivel.setPosition(0.361);
//            }


            //RIGHT BUMPER SAMPLE PICKUP AND SCORE
            if (gamepad1.right_bumper && !rightbumperpressed) {
                rightbumperpressed = true;
                if (bumperstage > 3.9 && bumperstage < 6.9) {
//                    rightbumperpressed  = true; //or false?
                    bumperstage = 7;
                } else if (bumperstage > 12.9) {
//                    rightbumperpressed = true; //or true?
                    bumperstage = 1;
                } else {
                    bumperstage++;
                }
            } else if (!gamepad1.right_bumper) {
                rightbumperpressed = false;
            }

            //RIGHT TRIGGER SPECIMEN PICKUP AND DROPOFF INTO OBSERVAITON ZONE
            if (gamepad1.right_trigger > 0.5 && !righttriggerpressed) {
                righttriggerpressed = true;
                if (bumperstage > 5.1) {
//                    rightbumperpressed = false; //or true?
                    bumperstage = 1;
                } else {
                    bumperstage++;
                }
            } else if (gamepad1.right_trigger < 0.5) {
                righttriggerpressed = false;
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
                    caseonepos = 1505; //if robot is out of size decrease this
                    break;
            }

            switch (bumperstage) {
                case 1: //slides and servos position for observation zone entry
                    //set claw mech positions in order to be able to move in and out of sub but as low as possible
                    //   leftslides.setTarget(1480);
                    submersibleinandout();
                    pivotposition(5, 0.35);
//                    slides(caseonepos);
                    leftslides.setTarget(caseonepos);
//                    slidespositio(caseonepos);
                    break;

                case 2: //slides and servos position once in observation zone
                    //once sample is located and have top of claw go down enough to touch sample and close claw becomes part of stage
                    samplepickuppos();
                    claw.setPosition(0.461);
                    leftslides.setTarget(caseonepos);
                    break;

                case 3: //close claw, reset claw mech position, retract slides (not fully). (claw will be closed seperately to make sure that sample is picked up)
                    clawtoggle = false;
                    leftslides.setTarget(caseonepos);
                    break;

                case 4:
                    clawtoggle = false; //close
                    clawswivel.setPosition(0.22);
                    resetpos();
//                    slides(50);
                    leftslides.setTarget(40);
//                    slidespositio(50);
                    break;

                case 5: //extend slides fully to drop sample into observation zone
//                    slides(1490);
                    leftslides.setTarget(1490);
//                    slidespositio(1490);
                    pivotposition(5, 0.35);
                    break;

                case 6: //open claw and retract slides to position ~500
                    clawtoggle = true; //open
//                    slides(700);
                    leftslides.setTarget(700);
//                    slidespositio(700);
                    pivotposition(5, 0.35);
                    break;

                case 7: //make slides vertical, fix claw mech position for sample release into basket, extend slides FULLY. make sure servos move simultaneously with other mechs
                    samplebasketrelease(); //check positions
                    pivotposition(268, 1);
                    break;

                case 8: //automatically happens
                    samplebasketrelease();
                    clawswivel.setPosition(0.22);
//                    slides(2374);
                    leftslides.setTarget(2374); //increase to 2390 if new belt is attached
//                    slidespositio(2374);
                    pivotposition(268, 1);
                    leftpivot.setPower(0.5);
                    rightpivot.setPower(0.5);
                    leftpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;

                case 9: //open claw, retract slides, horizontal pivot
                    clawtoggle = true; //open
                    pivotposition(268, 1);
                    break;

                case 10:
//                    slides(20);
                    leftslides.setTarget(40);
                    pivotposition(268, 1);
//                    slidespositio(20);
                    clawswivel.setPosition(0.22);
                    caseonepos = 200;
                    break;

                case 11: //automatically happens
                    pivotposition(10, 1);
                    caseonepos = 210;
                    break;
            }
            telemetry.addData("Sample Stage", bumperstage);
            telemetry.update();
//            if (bumperstage == 7 /** && sensor detects basket **/ ) {
//                clawtoggle = true; //open
//                bumperstage = 8;
//            }
//            if (bumperstage == 7 && (leftpivot.getCurrentPosition() > 134)) {
//                bumperstage = 8;
//            }
            if (bumperstage == 11 && (leftpivot.getCurrentPosition() < 20)) {
                bumperstage = 1;
            }
            if ((bumperstage == 7 || bumperstage == 8 || bumperstage == 9) && leftpivot.getCurrentPosition() < 255) {
                pivotposition(270, 1);
                //if it tries to fight against itself (not like how it can still fall now) then delete the following 6 lines
                leftpivot.setTargetPosition(270);
                rightpivot.setTargetPosition(270);
                leftpivot.setPower(1);
                rightpivot.setPower(1);
                leftpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //move automatically from stage 7 to stage 1 when slides are horizontal and retracted
//            if (bumperstage == 10 && (leftpivot.getCurrentPosition() < 50)) {
//                bumperstage = 1;
//            }

//                can only be done in stage 2 (maybe also stage 1 will have test to find out)
//                if (gamepad1.right_stick_y > 0.3) { //medium
//                    leftslides.setTarget(1525);
//                }
//                if (gamepad1.right_stick_y < -0.3) { //little
//                    leftslides.setTarget(900);
//                }

//                if (gamepad1.y) { //need to add if button y is pressed boolean
//                    bumperstage--;
//                }
            if (gamepad1.right_stick_button) {
                bumperstage = 1;
                clawtoggle = true; //open
            }

            //LEFT TRIGGER - SPECIMEN SCORING
            if (gamepad1.left_trigger > 0.5 && !lefttriggerpressed) {
                lefttriggerpressed = true;
                if (lefttriggerstage > 6) {
                    lefttriggerstage = 1;
                } else {
                    lefttriggerstage++;
                }
            } else if (gamepad1.left_trigger < 0.5) {
                lefttriggerpressed = false;
            }
            /**
             * stage 1 - pickup position - slides, servos, pivot
             * stage 2 - pickup and vertical and move servos into position for release
             * stage 3 - (automatic) extend slides
             * stage 4 - move slides up a bit, release claw
             * stage 5 - (atuomatic) retract slides fully,
             * OR
             * stage 1 - pickup position - slides, servos, pivot
             * stage 2 - claw close
             * stage 3 - (automatic) vertical and move servos into position for release
             * stage 4 - automatic - extend slides
             * stage 4 - open claw, move servos out of way and retract slides
             * stage 5 - (automatic) horizontal pivot
             *
             * PUT SPECIMEN STAGES INTO BUMPER FSM OR MAKE SPECIMEN ONE BUTTON TO RELOCALIZE AND  SCORE AUTONOMOUSLY
             * **/
            switch (lefttriggerstage) {
                case 1:
//                    slides(5);
                    leftslides.setTarget(5);
                    pivotposition(5, 1);
                    largeclawrotate(0.12);
                    singleclawrotate.setPosition(0.48);
                    clawswivel.setPosition(0.18);
                    break;

                case 2:
                    clawtoggle = false;
                    break;

                case 3:
                    singleclawrotate.setPosition(0.9);
                    break;

                case 4:
                    clawtoggle = false;
                    largeclawrotate(0.15);
                    clawswivel.setPosition(0.22);
                    pivotposition(270, 1);
                    break;

                case 5:
//                    slides(1635);
                    leftslides.setTarget(1635);
                    break;

                case 6:
                    clawtoggle = true;

                    break;
                case 7:
//                    slides(5);
                    leftslides.setTarget(5);
                    break;

            }
            if (lefttriggerstage == 2 && (claw.getPosition() > 0.455)) {
                lefttriggerstage = 3;
            }
            if (lefttriggerstage == 3 && (singleclawrotate.getPosition() > 0.8)) {
                lefttriggerstage = 4;
            }
//            if (lefttriggerstage == 4 && (leftpivot.getCurrentPosition() > 132)) {
//                lefttriggerstage = 5;
//            }
            if (lefttriggerstage == 6 && (claw.getPosition() < 0.1615)) {
                lefttriggerstage = 7;
            }
            if (lefttriggerstage == 7 && (leftslide.getCurrentPosition() < 50)) {
                lefttriggerstage = 1;
            }


            //reset position
            if (gamepad1.dpad_up) {
                leftslides.setTarget(0);
                pivotposition(1, 0.8);
            }

            if (gamepad1.dpad_down) {
                leftslides.setTarget(1635);
                pivotposition(1, 0.8);
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
                    leftslides.setTarget(1500);
//                    slides(1500);
                    pivotposition(268, 1);
                    break;
                case 3: //extend slides fully and *then* move pivot to be able to get onto high bar
                    pivotposition(155, 1);
                    break;
                case 4: //retract slides so robot can hang off high bar then release ascent servos from low bar and retract slides rest of the way. then put ascent servos back and release slides
                    leftpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    rightpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    leftslides.setTarget(100);
//                    slides(100);
                    break;
                case 5:
                    leftpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    rightpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    pivotposition(240, 0.7);
                    ascentstage++;
                    break;
                case 6:
                    //ascent servos move to position
                    leftpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    rightpivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    break;
                case 7:
                    pivotposition(270, 0.75);
                    leftslides.setTarget(1500);
//                    slides(1500);
                    break;
                case 8:
                    pivotposition(220, 1);
                    break;
                case 9:
                    leftslides.setTarget(100);
//                    slides(100);
                    break;
                case 10:
                    //move ascent servos to position
                    break;
            }

            if (ascentstage == 3 && leftpivot.getCurrentPosition() < 160) {
                ascentstage = 4;
            }
            if (ascentstage == 4 && leftslide.getCurrentPosition() < 150) {
                ascentstage = 5;
            }
//            if (ascentstage == 6 && ascentservo.getposition() > 0.5) {
//                ascentstage = 7;
//            }



                if (gamepad2.dpad_down) {
                    leftclawrotate.setPosition(0);
                    rightclawrotate.setPosition(0);
                }
                if (gamepad2.y) {
                    leftclawrotate.setPosition(1);
                    rightclawrotate.setPosition(1);
                }
                if (gamepad2.x) {
                    singleclawrotate.setPosition(0.5);
                }
                if (gamepad2.b) {
                    claw.setPosition(1);
                }
            if (gamepad2.dpad_up) {
                singleclawrotate.setPosition(0);
            }

            //DRIVER STUFF
            //later convert this to pedro

            if (gamepad2.left_bumper && !driverleftbumper && speedx > 0) {
                speedx--;
                driverleftbumper = true;
            } else if (!gamepad2.left_bumper) {
                driverleftbumper = false;
            }
            if (gamepad2.right_bumper && !driverrightbumper && speedx < 2) {
                speedx++;
                driverrightbumper = true;
            } else if (!gamepad2.right_bumper) {
                driverrightbumper = false;
            }
            telemetry.addData("drivespeed:", speed);
            telemetry.update();


            rx = gamepad2.left_stick_y; // Remember, Y stick value is reversed
            x = -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            y = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);

            frontLeftPower = speed[speedx] * ((y + x + rx) / denominator);
            backLeftPower = speed[speedx] * ((y - x + rx) / denominator);
            frontRightPower = speed[speedx] * ((y - x - rx) / denominator);
            backRightPower = speed[speedx] * ((y + x - rx) / denominator);

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
//bumperstage == 1 || bumperstage == 2 || bumperstage == 3 || bumperstage == 4 || bumperstage == 5 ||
            if (bumperstage == 1 || bumperstage == 2 || bumperstage == 3 || bumperstage == 4 || bumperstage == 5 || bumperstage == 6 || bumperstage == 7 || bumperstage == 8 || bumperstage == 9 || bumperstage == 11 || ascentstage == 2 || ascentstage == 3 || ascentstage == 4 || ascentstage == 5 || ascentstage == 6 || ascentstage == 7) {
                leftslides.armTask();
            }

        }
    }
}