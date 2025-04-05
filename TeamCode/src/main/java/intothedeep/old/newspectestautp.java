package intothedeep.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="newtest", group="Linear OpMode")
public class newspectestautp extends LinearOpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor arm;
    CRServo intake;
    Servo swivel;


    public void resetencoders()
    {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void drivetrain(int desiredPosition) {
        FL.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition
        FR.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition
        BL.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition
        BR.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition

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

    public void leftturn(int desiredpos) {
        FL.setTargetPosition(-desiredpos); // Tells the motor that the position it should go to is desiredPosition
        FR.setTargetPosition(desiredpos); // Tells the motor that the position it should go to is desiredPosition
        BL.setTargetPosition(-desiredpos); // Tells the motor that the position it should go to is desiredPosition
        BR.setTargetPosition(desiredpos); // Tells the motor that the position it should go to is desiredPosition

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && FL.isBusy() || FR.isBusy() || BR.isBusy() || BL.isBusy()){
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

    public void rightturn(int desiredpos) {
        FL.setTargetPosition(desiredpos); // Tells the motor that the position it should go to is desiredPosition
        FR.setTargetPosition(-desiredpos); // Tells the motor that the position it should go to is desiredPosition
        BL.setTargetPosition(desiredpos); // Tells the motor that the position it should go to is desiredPosition
        BR.setTargetPosition(-desiredpos); // Tells the motor that the position it should go to is desiredPosition

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && FL.isBusy() || FR.isBusy() || BR.isBusy() || BL.isBusy()){
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

    public void individual(int frontleft, int frontright, int backleft, int backright) {
        FL.setTargetPosition(frontleft); // Tells the motor that the position it should go to is desiredPosition
        FR.setTargetPosition(frontright); // Tells the motor that the position it should go to is desiredPosition
        BL.setTargetPosition(backleft); // Tells the motor that the position it should go to is desiredPosition
        BR.setTargetPosition(backright); // Tells the motor that the position it should go to is desiredPosition

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
    public void arm(int desiredPosition) {
        arm.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()){
            arm.setPower(0.5);
        }
        arm.setPower(0);

    }



    public void runOpMode() {
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

        // Reset the motor encoder so that it reads zero ticks
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double CPR = 537.7;
        int armCPR = 8192;

        double diameter = 9.6; // Replace with your wheel/spool's diameter
        double circumference = Math.PI * diameter;
        int drivetrainconvert = 17;
        int armconvert = -23;

        waitForStart();

        if (opModeIsActive()) {

            drivetrain(25*drivetrainconvert); // drive backwards
            resetencoders();
            arm(-130*drivetrainconvert);
            swivel.setPosition(0.108);
            drivetrain(63*drivetrainconvert);
            resetencoders();
            intake.setPower(-1);
            arm(-100*drivetrainconvert);
            drivetrain(-14*drivetrainconvert);
            intake.setPower(1);


//            leftturn(18*drivetrainconvert); //turn left
//            resetencoders();
//            arm(75*armconvert); //raise the arm half way up
//            swivel.setPosition(0.12); // change swivel to accurate orientation
//            arm(150*armconvert); //raise arm to fully position
//            drivetrain(-7*drivetrainconvert);  //drive back to adjust
//            resetencoders();
//            arm(170*armconvert); // arm up (over 180 line) further to adjust
//            drivetrain(7*drivetrainconvert); //drive forward to adjust
//            resetencoders();
//            arm(180*armconvert); //final up
//            drivetrain(15*drivetrainconvert); // clip and release (was 15)
//            resetencoders();
//            intake.setPower(1);
//            sleep(2000);
//            intake.setPower(0);
//
//            arm(0);
//            rightturn(20*drivetrainconvert);
//            resetencoders();
//            drivetrain(20*drivetrainconvert);
//            resetencoders();
//            individual(100*drivetrainconvert,-100*drivetrainconvert,-100*drivetrainconvert,100*drivetrainconvert);
//            resetencoders();






// forward
            // down
            ///outtake
            //forward



        }

    }


}
