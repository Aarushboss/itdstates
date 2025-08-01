package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class timebasedauto extends LinearOpMode {

    DcMotorEx leftdrive, rightdrive, arm;
    Servo leftclaw, rightclaw;

    public void arm(int desiredPosition) {

        arm.setTargetPosition(desiredPosition);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        while(arm.isBusy())
            arm.setPower(0.5);
        arm.setPower(0);

    }

    public void runOpMode(){


        leftdrive = hardwareMap.get(DcMotorEx.class, "leftdrive");
        rightdrive = hardwareMap.get(DcMotorEx.class, "rightdrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");

        leftdrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftclaw.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            leftdrive.setPower(0.5);
            rightdrive.setPower(0.5);
            sleep(1000);
            leftdrive.setPower(0);
            rightdrive.setPower(0);
            arm(100);
        }

}}
