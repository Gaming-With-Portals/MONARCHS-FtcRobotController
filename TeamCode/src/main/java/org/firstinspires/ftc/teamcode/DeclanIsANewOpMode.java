package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Declan's Final Reckoning", group="Linear OpMode")
@Configurable
public class DeclanIsANewOpMode extends LinearOpMode {

    public static TheMonarch robot = new TheMonarch();

    boolean intake_active = false;
    boolean outake_active = false;

    boolean right_door_open = false;
    boolean left_door_open = false;
    boolean slow_mode = false;

    boolean injector_active = false;

    boolean is_chambered = false;

    @Override
    public void runOpMode() {

        robot.Init(hardwareMap);
        telemetry.addData("Status:", "READY");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status:", "ACTIVE");
        telemetry.update();
        robot.Reset();

        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;

            double yaw     =  gamepad1.right_stick_x;

            if (gamepad1.left_trigger>0.05){ // deadzone
                telemetry.addData("AprilTag Status:","Active");
                robot.CameraDebugData(telemetry);
                for (AprilTagDetection detection : robot.FetchAprilTagDetections()){
                    if (detection.id == 20){ // TODO: add red target
                        yaw = detection.ftcPose.yaw * 0.015;
                    }
                }

            }

            robot.TeleOpDrive(axial, lateral, yaw);

            if (gamepad1.aWasPressed()){
                intake_active=!intake_active;
                robot.ToggleIntake(intake_active);
            }

            if (gamepad1.bWasPressed()){
                injector_active = !injector_active;
                robot.ToggleInjectorBar(injector_active);
            }

            if (gamepad1.xWasPressed()){
                outake_active = !outake_active;
                robot.ToggleLaunchMotor(outake_active);
            }

            if (gamepad1.yWasPressed()){
                is_chambered = !is_chambered;
                robot.ToggleChamber(is_chambered);
            }

            if (gamepad1.rightStickButtonWasPressed()){
                slow_mode = !slow_mode;
                if (slow_mode){
                    robot.speedMultiplier = 0.25;
                }else{
                    robot.speedMultiplier = 1;
                }
            }

            if (gamepad1.dpadRightWasPressed()){
                right_door_open = !right_door_open;
                robot.ToggleRightDoor(right_door_open);
                if (right_door_open && left_door_open){
                    robot.ToggleLeftDoor(false);
                    left_door_open = false;
                }
            }

            if (gamepad1.dpadLeftWasPressed()){
                left_door_open = !left_door_open;
                robot.ToggleLeftDoor(left_door_open);
                if (right_door_open && left_door_open){
                    robot.ToggleRightDoor(false);
                    right_door_open = false;
                }
            }

            telemetry.addData("R:", robot.chamber_color.red());
            telemetry.addData("G:", robot.chamber_color.blue());
            telemetry.addData("B:", robot.chamber_color.green());

            telemetry.update();
        }

    }
}
