package intothedeep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Important: Use DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Linear Slide PIDF Setup")
public class LinearSlidePIDFSetup extends LinearOpMode {

    private DcMotorEx slideMotor1;
    private DcMotorEx slideMotor2;


    public static double SLIDE_POSITION_P = 5.0; // if too slow, then increase. if too fast/oscillating, then decrease.

    public static double SLIDE_VELOCITY_P = 0.0055;
    public static double SLIDE_VELOCITY_I = 0;
    public static double SLIDE_VELOCITY_D = 0.001;
    public static double SLIDE_VELOCITY_F = 0.06;


    @Override
    public void runOpMode() {

        slideMotor1 = hardwareMap.get(DcMotorEx.class, "slideMotor1");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "slideMotor2");

        // slideMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        // slideMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SLIDE_VELOCITY_P, SLIDE_VELOCITY_I, SLIDE_VELOCITY_D, SLIDE_VELOCITY_F);
        slideMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        slideMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        slideMotor1.setPositionPIDFCoefficients(SLIDE_POSITION_P);
        slideMotor2.setPositionPIDFCoefficients(SLIDE_POSITION_P);

        // slideMotor1.setTargetPosition(0); // Example target
        // slideMotor2.setTargetPosition(0); // Example target
        // slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        if (opModeIsActive()) {

            int targetPosition = 1000;
            slideMotor1.setTargetPosition(targetPosition);
            slideMotor2.setTargetPosition(targetPosition);

            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor1.setPower(0.8);
            slideMotor2.setPower(0.8);

            // Loop while the motors are busy moving to the target
            while (opModeIsActive() && (slideMotor1.isBusy() || slideMotor2.isBusy())) {
                telemetry.addData("Slide 1 Pos", slideMotor1.getCurrentPosition());
                telemetry.addData("Slide 1 Target", slideMotor1.getTargetPosition());
                telemetry.addData("Slide 2 Pos", slideMotor2.getCurrentPosition());
                telemetry.addData("Slide 2 Target", slideMotor2.getTargetPosition());
                telemetry.update();
                idle();
            }


            slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Motors reached target or OpMode stopped.");
            telemetry.update();
        }
    }
}
