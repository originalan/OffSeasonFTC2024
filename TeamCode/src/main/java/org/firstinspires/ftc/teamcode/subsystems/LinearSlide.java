package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotSettings.MAX_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BulkReading;
import org.firstinspires.ftc.teamcode.MotionProfile;
import org.firstinspires.ftc.teamcode.PIDFControl;

public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private PIDFControl controller;
    private ElapsedTime motionProfileTime = new ElapsedTime();
    private int STARTING_POS = 0, ENDING_POS = 0;

    public enum SlideState {
        MOTION_PROFILE,
        REST,
        NOTHING
    }
    public SlideState slideState = SlideState.REST;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        mp = new MotionProfile();
        controller = new PIDFControl(telemetry);
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Slide Motor Position", robot.slideMotor.getCurrentPosition());
    }

    @Override
    public void update() {
        switch (slideState) {
            case NOTHING:
                break;
            case REST:
                break;
            case MOTION_PROFILE:
                mp.updateState(motionProfileTime.seconds());
                double refPos = mp.getInstantPosition();
                double refVel = mp.getInstantVelocity();
                double refAcl = mp.getInstantAcceleration();

                double power = controller.calculatePIDF(refPos, BulkReading.pSlideMotor);
                setSlidePower(power);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void setSlidePower(double power) {
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }
        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }

        robot.slideMotor.setPower(power);
    }

    public void setMotionProfile(int targetPosition) {
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motionProfileTime.reset();
        mp.setStartingTime(motionProfileTime.seconds());

        STARTING_POS = BulkReading.pSlideMotor;
        ENDING_POS = targetPosition;

        mp.setProfile(STARTING_POS, ENDING_POS);
    }

    public MotionProfile getMP() {
        return mp;
    }
}
