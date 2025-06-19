package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class summercampteleop extends LinearOpMode {

    DcMotorEx leftdrive, rightdrive, arm;
    Servo leftclaw, rightclaw;

    double leftstick, rightstick, leftbumper, rightbumper;

    @Override
    public void runOpMode() throws InterruptedException {

        leftdrive = hardwareMap.get(DcMotorEx.class, "leftdrive");
        rightdrive = hardwareMap.get(DcMotorEx.class, "rightdrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");
        rightclaw.setDirection(Servo.Direction.REVERSE);




        while(opModeIsActive()) {

            leftstick = gamepad1.left_stick_y;
            rightstick = gamepad1.right_stick_y;

            if (Math.abs(leftstick) > 0.1) {
                leftdrive.setPower(leftstick);
                rightdrive.setPower(rightstick);
            }

            leftbumper = gamepad1.left_trigger;
            rightbumper = gamepad1.right_trigger;

            if ((leftbumper > 0.1) || (rightbumper > 0.1)) {
                arm.setPower(leftbumper - rightbumper);
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
