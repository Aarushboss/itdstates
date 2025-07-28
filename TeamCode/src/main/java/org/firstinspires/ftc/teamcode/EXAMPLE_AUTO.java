package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous (name="example auto", group="Linear OpMode")
@Disabled
public class EXAMPLE_AUTO extends LinearOpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;


    //variables used to make values more readable and easy to use with encoders
    double CPR = 537.7;
    int armCPR = 8192;


    double diameter = 9.6; // Replace with your wheel/spool's diameter
    double circumference = Math.PI * diameter;
    int drivetrainconvert = 17;
    int armconvert = -23;


    public void resetencoders() {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //the following means that the program does not use pid feedback loop for accurate motor movements. this is necessary for encoder-based autonomous programs.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(int desiredPosition) {
        FL.setTargetPosition(desiredPosition*drivetrainconvert); // Tells the motor that the position it should go to is desiredPosition
        FR.setTargetPosition(desiredPosition*drivetrainconvert); // Tells the motor that the position it should go to is desiredPosition
        BL.setTargetPosition(desiredPosition*drivetrainconvert); // Tells the motor that the position it should go to is desiredPosition
        BR.setTargetPosition(desiredPosition*drivetrainconvert); // Tells the motor that the position it should go to is desiredPosition

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FL.isBusy() || FR.isBusy() || BR.isBusy() || BL.isBusy()){
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "motor");
        FR = hardwareMap.get(DcMotor.class, "motor");
        BL = hardwareMap.get(DcMotor.class, "motor");
        BR = hardwareMap.get(DcMotor.class, "motor");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        waitForStart();


        if (opModeIsActive()) {

            drive(30);
            resetencoders();
            drive(-30);
            resetencoders();


        }


    }




}
