package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp
public class anotherteleop extends LinearOpMode {

    DcMotorEx leftdrive, rightdrive, arm;
    Servo leftclaw, rightclaw;

//    double leftstick, rightstick, leftbumper, rightbumper;

    @Override
    public void runOpMode() throws InterruptedException {

        leftdrive = hardwareMap.get(DcMotorEx.class, "leftdrive");
        rightdrive = hardwareMap.get(DcMotorEx.class, "rightdrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");
        rightclaw.setDirection(Servo.Direction.REVERSE);




        while(opModeIsActive()) {

//            leftstick = gamepad1.left_stick_y;
//            rightstick = gamepad1.right_stick_y;

            if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) {
                leftdrive.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1) {
                rightdrive.setPower(gamepad1.right_stick_y);
            }


            if ((gamepad2.left_bumper)) {
                arm.setPower(-1);
            } else if ((gamepad2.right_bumper)) {
                arm.setPower(1);
            } else {
                arm.setPower(0);
            }
//HI
            if (gamepad1.a) { //close
                leftclaw.setPosition(0.5);
                rightclaw.setPosition(0.5);
            }

            if (gamepad1.b) { //open
                leftclaw.setPosition(0);
                rightclaw.setPosition(0);
            }

//Sandy Was Here :)
        }
    }

}
//ex teleops for summer camp robots