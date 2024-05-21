package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BulkReading;
import org.firstinspires.ftc.teamcode.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;

@Config
@TeleOp (name = "Slide Test Basic", group = "Testing")
public class SlideTestBasic extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    public static int GOAL_POSITION = 0;

    private double doubleCheckWaitTime = 0;

    public enum SlideControlState {
        OFF,
        GO_TO_POSITION,
        RESET
    }
    public SlideControlState slideControlState = SlideControlState.OFF;
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed Time", runtime.toString());
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive()) {

                bulkReading.readAll();
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                slideControls();

                robot.addTelemetry();
                telemetry.addLine("Press Y to go to position, y again to turn off");
                telemetry.addLine("Press dpad down to reset encoders and stuff");
                telemetry.update();
                robot.update();
            }
        }

    }

    public void slideControls() {
        switch(slideControlState) {
            case OFF:
                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.REST;
                    slideControlState = SlideControlState.GO_TO_POSITION;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    slideControlState = SlideControlState.RESET;
                }
                break;
            case GO_TO_POSITION:
                double power = robot.slideSubsystem.controller.calculatePIDF(GOAL_POSITION, BulkReading.pSlideMotor);
                robot.slideSubsystem.setSlidePower(power);
                if (currentGamepad1.y && !previousGamepad1.y) {
                    slideControlState = SlideControlState.OFF;
                }
                break;
            case RESET:
                robot.slideMotor.setPower(0);
                robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.slideSubsystem.slideState = LinearSlide.SlideState.REST;

                slideControlState = SlideControlState.OFF;
                break;
        }
    }

}
