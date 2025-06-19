package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
@Disabled
public class pidtest extends OpMode {
    public PIDController controller;

    //for slides
    public static double p = 0.0055, i = 0, d = 0.001;
    public static double f = 0.1;

//    public static double p = 0, i = 0, d = 0;
//    public static double f = 0;

    public static int slidestarget;

    private final double ticks_in_degree = 1425.1/360; //this should be right?

    private DcMotorEx leftslide;
    private DcMotorEx rightslide;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        rightslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftslide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightslide.setDirection(DcMotorEx.Direction.REVERSE);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int rightslidepos = rightslide.getCurrentPosition();
        int leftslidepos = leftslide.getCurrentPosition();
//        int slidepos = (rightslidepos + leftslidepos)/2;
        double pid = controller.calculate(rightslidepos, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget/ticks_in_degree)) * f;

        double power = pid + ff;

        leftslide.setPower(power);
        rightslide.setPower(power);

        telemetry.addData("rightpos ", rightslidepos);
        telemetry.addData("leftpos ", leftslidepos);
        telemetry.addData("targetangle", slidestarget);
        telemetry.update();
    }
}