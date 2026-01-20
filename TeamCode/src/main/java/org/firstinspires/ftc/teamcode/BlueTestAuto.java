
package org.firstinspires.ftc.teamcode;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@Autonomous(name = "Blue Auto", group = "Autonomous")
@Configurable // Panels
public class BlueTestAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer;

    private DcMotor launch_motor = null;
    private DcMotor intake_motor = null;
    private DcMotor injection_motor = null;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    //private final Pose scorePose = new Pose(50, 100, Math.toRadians(130));
    private final Pose scorePose = new Pose(35, 100, Math.toRadians(130));

    private final Pose midPose = new Pose(45, 45, Math.toRadians(90));

    private PathChain toShoot;

    private PathChain toHome;

    public static double launchSpeed = 0.65;


    @Override
    public void init() {
        pathTimer = new Timer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        launch_motor = hardwareMap.get(DcMotor.class, "outake");
        injection_motor = hardwareMap.get(DcMotor.class, "injector");

        launch_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        injection_motor.setDirection(DcMotorSimple.Direction.REVERSE);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        follower.setMaxPower(1);

        toShoot = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, scorePose))
                                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                                        .build();

        toHome = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {



        public Paths(Follower follower) {


        }
    }


    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(toShoot);

                setPathState(1);


                break;
            case 1:
                telemetry.addData("Elapsed: ", pathTimer.getElapsedTimeSeconds());
                if (!follower.isBusy()){

                    if (pathTimer.getElapsedTimeSeconds() > 10){
                        injection_motor.setPower(0);
                        intake_motor.setPower(0);
                        launch_motor.setPower(0);

                        setPathState(2);


                    }else if (pathTimer.getElapsedTimeSeconds() > 8){
                        injection_motor.setPower(1);
                        intake_motor.setPower(1);
                    }else{
                        launch_motor.setPower(launchSpeed);
                    }
                }


                break;

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(toHome);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(0);
                }
                break;




        }

        return pathState;
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    