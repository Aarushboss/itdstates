package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class oldteleop extends LinearOpMode {
    //Custom variable declaration for button speed change
    public static final double[] speed = {0.15, 0.45, 0.95};



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor FL;
        DcMotor FR;
        DcMotor BL;
        DcMotor BR;
        DcMotor arm;
        CRServo intake;
        Servo swivel;

        //

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        FL = hardwareMap.get(DcMotor.class, "frontleft");
        FR = hardwareMap.get(DcMotor.class, "frontright");
        BL = hardwareMap.get(DcMotor.class, "backleft");
        BR = hardwareMap.get(DcMotor.class, "backright");
        arm = hardwareMap.get(DcMotor.class, "mainarmmotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        swivel = hardwareMap.get(Servo.class, "swivel");


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Custom variable declaration for button speed change
        byte speedx = 2;
        waitForStart();
        //Set Arm position
        if (isStopRequested()) return;
        //arm cannont move from here


        while (opModeIsActive()) {

            //final double CPR = 537.7;
            // Get the current position of the motor
            int position = arm.getCurrentPosition();
            // double revolutions = position/CPR;

            //double angle = revolutions * 360;
            //double angleNormalized = angle % 360;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            double frontLeftPower = speed[speedx] * ((y + x + rx) / denominator);
            double backLeftPower = speed[speedx] * ((y - x + rx) / denominator);
            double frontRightPower = speed[speedx] * ((y - x - rx) / denominator);
            double backRightPower = speed[speedx] * ((y + x - rx) / denominator);
            double armPower = gamepad2.left_stick_y;

            //hell nightmare SIMAHEMAYEYA type arm stopper



            if(gamepad2.right_bumper)
            {
                intake.setPower(1);
            }
            else if (gamepad2.a)
            {
                intake.setPower(0);
            } else if (gamepad2.left_bumper)
            {
                intake.setPower(-1);
            }




            if (gamepad2.b) swivel.setPosition(0.001);
            else if (gamepad2.y) swivel.setPosition(0.06);
            else if (gamepad2.x) swivel.setPosition(0.118); //d




            if (-5850 > position && gamepad2.left_stick_y < 0) { // backward limit
                armPower = 0;
            }
            if (-150 < position && gamepad2.left_stick_y > 0 ) { // front limit
                armPower = 0;
            }

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
            arm.setPower(armPower);
        }
    }
}


