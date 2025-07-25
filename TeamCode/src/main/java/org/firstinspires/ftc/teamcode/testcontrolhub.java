package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testcontrolhub extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo t = hardwareMap.get(Servo.class, "t");
        Servo e = hardwareMap.get(Servo.class, "e");
        Servo s = hardwareMap.get(Servo.class, "s");
        Servo p = hardwareMap.get(Servo.class, "p");
        Servo h = hardwareMap.get(Servo.class, "h");
        Servo o = hardwareMap.get(Servo.class, "o");
        //tespho

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                t.setPosition(0.5);
                e.setPosition(0.5);
                s.setPosition(0.5);
                p.setPosition(0.5);
                h.setPosition(0.5);
                o.setPosition(0.5);
            } else if (gamepad1.b) {
                t.setPosition(0);
                e.setPosition(0);
                s.setPosition(0);
                p.setPosition(0);
                h.setPosition(0);
                o.setPosition(0);
            }

        }
    }
}
