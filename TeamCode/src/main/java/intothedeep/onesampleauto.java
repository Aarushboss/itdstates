package intothedeep;

import static java.lang.Math.abs;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.constants.PinpointConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous (preselectTeleOp = "newteleop")
public class onesampleauto extends LinearOpMode {
    public static final double[] speed = {0.25, 0.65, 1};

    double subinout = 0.41;
    double inandout = 0.20;

    private Timer ascenttimer;
    DcMotorEx leftslide, rightslide;
    DcMotorEx leftpivot, rightpivot;
    DcMotorEx FL, FR, BL, BR;
    Servo claw, clawswivel, singleclawrotate, leftclawrotate, rightclawrotate;

    public static double clawswivelcenter = 0.181, clawswivelleft = 0.6, clawswivelright = 0;

    final static double p = 0.0055, i = 0, d = 0.0001;
    double f = 0.1;
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

    public void drive(int dist) {
        FL.setPower(dist);
        BL.setPower(dist);
        FR.setPower(dist);
        BR.setPower(dist);
    }
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
        singleclawrotate.setPosition(0.7); //figure out what to set this
    }

    public void submersibleinandout() {
        largeclawrotate(inandout);
        singleclawrotate.setPosition(0.15); //figure out what to set this
    }

    public void samplepickuppos() {
        largeclawrotate(0.07);
        singleclawrotate.setPosition(0.17); //figure out what to set this
    }

    public void resetpos() {
        largeclawrotate(0.5);
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
        FL.setDirection(DcMotorEx.Direction.REVERSE);



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

        if (opModeIsActive()) {



            claw.setPosition(0.1615); //close
            pivotposition(270,0.9);
//            drive(-1);
            FL.setPower(-1);
            BL.setPower(-1);
            FR.setPower(-1);
            BR.setPower(-1);
            sleep(300);
//            drive(0);
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            leftslides.armTask();
            samplebasketrelease();
            leftslides.setTarget(2381);
            sleep(1000);
            leftslides.armTask();
//            drive(-1);
            FL.setPower(-1);
            BL.setPower(-1);
            FR.setPower(-1);
            BR.setPower(-1);
            sleep(40);
//            drive(0);
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            samplebasketrelease();
            claw.setPosition(0.5); //open
//            drive(1);
            FL.setPower(1);
            BL.setPower(1);
            FR.setPower(1);
            BR.setPower(1);
            sleep(500);
//            drive(0);
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            leftslides.armTask();
            leftslides.setTarget(10);
            leftslides.armTask();
            pivotposition(20, 0.8);








            telemetry.update();

            if (bumperstage == 1 || bumperstage == 2 || bumperstage == 3 || bumperstage == 4 || bumperstage == 8 || bumperstage == 9 || bumperstage == 10 || ascentstage == 2 || ascentstage == 3 || ascentstage == 4 || ascentstage == 5 || ascentstage == 6 || ascentstage == 7) {
                leftslides.armTask();
            }

        }
    }
}