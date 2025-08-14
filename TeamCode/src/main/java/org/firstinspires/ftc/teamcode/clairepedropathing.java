package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous(name = "clairepedrotestingauto", group = "Examples")
public class clairepedropathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(8,70, Math.toRadians(0.0));
    private final Pose poseuna = new Pose(30, 70, Math.toRadians(0));
    private final Pose posedos = new Pose(25, 26, Math.toRadians(0));
    private final Pose posetres = new Pose(50, 33.5, Math.toRadians(0));
    private final Pose posecuatro = new Pose(50, 12.948, Math.toRadians(0));
    private final Pose posecinco = new Pose(-20, 35, Math.toRadians(0));
    private final Pose poseseis = new Pose(50, 25.856, Math.toRadians(0));

    private PathChain triangle;

    private Telemetry telemetryA;

    PathBuilder builder = new PathBuilder();

    int pathState;

    private Path pathone;
    private PathChain pathtwo, paththree, pathfour, pathfive;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }

        follower.telemetryDebug(telemetryA);
    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(poseuna)))
                .setLinearHeadingInterpolation(startPose.getHeading(), poseuna.getHeading())
                .addPath(new BezierCurve(new Point(poseuna), new Point(posedos), new Point(posetres)))
                .setLinearHeadingInterpolation(posedos.getHeading(), posetres.getHeading())
                .addPath(new BezierCurve(new Point(posetres), new Point(posecuatro), new Point(posecinco), new Point(poseseis)))
                .setLinearHeadingInterpolation(posecuatro.getHeading(), poseseis.getHeading())
                .build();

        follower.followPath(triangle);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}