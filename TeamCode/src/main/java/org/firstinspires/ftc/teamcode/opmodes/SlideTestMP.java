package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp (name = "Slide Test MP", group = "Testing")
public class SlideTestMP extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private double doubleCheckWaitTime = 0;

    public enum SlideControlState {
        OFF,
        GO_TO_POSITION,
        GO_BACK_DOWN,
        DOUBLE_CHECK,
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
                telemetry.update();
                robot.update();
            }
        }

    }

    public void slideControls() {
        switch(slideControlState) {
            case OFF:
                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.MOTION_PROFILE;
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_2);
                    slideControlState = SlideControlState.GO_TO_POSITION;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.MOTION_PROFILE;
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_3);
                    slideControlState = SlideControlState.GO_TO_POSITION;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.MOTION_PROFILE;
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_4);
                    slideControlState = SlideControlState.GO_TO_POSITION;
                }
                break;
            case GO_TO_POSITION:
                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_2);
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_3);
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_4);
                }

                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.slideSubsystem.setMotionProfile(RobotSettings.SLIDE_POS_1);
                    slideControlState = SlideControlState.GO_BACK_DOWN;
                }
                break;
            case GO_BACK_DOWN:
                if (!robot.slideSubsystem.getMP().isBusy()) {
                    robot.slideMotor.setPower(0);
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.REST;

                    doubleCheckWaitTime = runtime.seconds();
                    slideControlState = SlideControlState.DOUBLE_CHECK;
                }
                break;
            case DOUBLE_CHECK:
                if (runtime.seconds() - doubleCheckWaitTime > 0.4) {
                    slideControlState = SlideControlState.RESET;
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
