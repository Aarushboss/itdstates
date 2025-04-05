package intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.examples.Triangle;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous
@Config
public class pedrotestauto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(8,70, Math.toRadians(0));
    private final Pose poseuna = new Pose(25, 70, Math.toRadians(0));
    private final Pose posedos = new Pose(23, 24, Math.toRadians(0));
    private final Pose posetres = new Pose(48, 30, Math.toRadians(0));
    private final Pose posecuatro = new Pose(42, 14.948, Math.toRadians(0));
    private final Pose posecinco = new Pose(-10, 33, Math.toRadians(0));
    private final Pose poseseis = new Pose(46, 24.856, Math.toRadians(0));

    private PathChain triangle;

    private Telemetry telemetryA;

    PathBuilder builder = new PathBuilder();

    int pathState;

    private Path pathone;
    private Path pathtwo, paththree, pathfour, pathfive;

    private PathChain pathChain;



    public void buildPaths() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pathone = new Path(new BezierLine(new Point(startPose), new Point(poseuna)));
        pathone.setLinearHeadingInterpolation(startPose.getHeading(), poseuna.getHeading());

        pathtwo = new Path(new BezierCurve(new Point(poseuna), new Point(posedos), new Point(posetres)));
        pathtwo.setLinearHeadingInterpolation(poseuna.getHeading(), posetres.getHeading());

        paththree = new Path(new BezierCurve(new Point(posetres), new Point(posecuatro), new Point(posecinco), new Point(poseseis)));
        paththree.setLinearHeadingInterpolation(posetres.getHeading(), poseseis.getHeading());

        pathChain = new PathChain(pathone, pathtwo, paththree);

//        pathtwo = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(poseuna), new Point(posedos), new Point(posetres)))
//                .setLinearHeadingInterpolation(posedos.getHeading(), posetres.getHeading())
//                .build();
//
//        paththree = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(posetres), new Point(posecuatro), new Point(posecinco), new Point(poseseis)))
//                .setLinearHeadingInterpolation(posecuatro.getHeading(), poseseis.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathone, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pathtwo, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paththree, false);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }


    @Override
    public void loop() {
        follower.update();
//        autonomousPathUpdate();

        follower.followPath(pathChain, false); //maube try false later

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower.telemetryDebug(telemetryA);
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
        setPathState(0);
    }

    @Override
    public void stop(){

    }

}