package intothedeep.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="zeroingservos", group="Linear OpMode")
@Disabled

public class zeroingservos extends LinearOpMode {

    Servo leftend, rightend;





    public void runOpMode() {

        leftend = hardwareMap.get(Servo.class, "leftend");
        rightend = hardwareMap.get(Servo.class, "rightend");





        waitForStart();

        if (opModeIsActive()) {





        }

    }


}
