package intothedeep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

class Arm {
    public final PIDController controller;
     private final double ticks_in_degree = 1425.1/360;
     private int target = 0;

    double p = 0.0055, i = 0, d = 0.001, f = 0.1;
    double pivotf = 0.2;

    private DcMotorEx leftslide; //these are private so we need to redeclare them in the teleop
    private DcMotorEx rightslide;
    private DcMotorEx leftpivot;
    private DcMotorEx rightpivot;
    private DcMotorEx arm_motor;

    private double power;



    /**
     * Constructor: create and initialize everything required by the arm subsystem including the arm motor and
     * PID controller.
     */

    public Arm(
            HardwareMap hardwareMap, String name, double p, double i, double d, double f)  //this is major change i made
    {
          // Initialize arm characteristics.
         // Create arm motor here and initialize it.
        arm_motor = hardwareMap.get(DcMotorEx.class, name);
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");

        power = 1;

//        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Create PID controller for the arm and initialize it with proper PID coefficients.
        controller = new PIDController(p, i, d);
    }


    /**
     * This method must be called periodically so that it will do PID control of the arm towards targetangle position and
     * hold it.
     //     */
//    public static void setPosition(int targetPos)
//    {
//        targetangle = targetPos;
//    }

    public void setTarget(int settarget) {
        target = settarget;
    }

    public void setPower(double power)
    {
        if (power > 0.0)
        {
            // Move arm towards max position with specified power.
            setTarget(1500);
        }
        else if (power < 0.0)
        {
            // Move arm towards min position with specified power.
            setTarget(0);
        }
        else
        {
            // Hold arm position without power limit.
            setTarget(leftslide.getCurrentPosition());
        }
    }

//    public void joystickarmtask() {
//
//        controller.setPID(p, i, d);
//        int slidepos = (leftslide.getCurrentPosition() + rightslide.getCurrentPosition())/ 2;
//        int pos = arm_motor.getCurrentPosition();
//        double pid = controller.calculate(pos, targetangle);
//
//        power = pid;
//
//        leftslide.setPower(power);
//        rightslide.setPower(power);
//
//    }

    public void armTask() {

        controller.setPID(p, i, d);
        int slidepos = (leftslide.getCurrentPosition() + rightslide.getCurrentPosition())/ 2;
        int pos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        power = pid + ff;

        leftslide.setPower(power);
        rightslide.setPower(power);

    }
}