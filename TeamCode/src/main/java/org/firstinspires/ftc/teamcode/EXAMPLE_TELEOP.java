package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
@Disabled
public class EXAMPLE_TELEOP extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors


        DcMotor motor;
        motor = hardwareMap.get(DcMotor.class, "motor");



        motor.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        if (isStopRequested()) return;



        while (opModeIsActive()) {
            //teleop goes on here  -- where you tell the robot exactly what to do during the program

            if (gamepad1.a) {
                motor.setPower(1);
            } else if (gamepad1.b) {
                motor.setPower(-1);
            } else {
                motor.setPower(0);
            }


        }
    }
}
