package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSettings;

import java.util.Arrays;
import java.util.List;

public class JVBoysSoccerRobot {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private List<LynxModule> allHubs;

    // Subsystems
    private List<Subsystem> subsystems;
    public LinearSlide slideSubsystem;

    // Hardware
    public DcMotorEx slideMotor;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initHardware();

        slideSubsystem = new LinearSlide(hwMap, telemetry, this);

        subsystems = Arrays.asList(slideSubsystem);

    }

    public void initHardware() {
        slideMotor = hwMap.get(DcMotorEx.class, RobotSettings.SLIDE_MOTOR_NAME);
        slideMotor.setDirection(RobotSettings.SLIDE_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void addTelemetry() {
        for (Subsystem s : subsystems) {
            s.addTelemetry();
        }
    }

    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void stop() {
        for (Subsystem s : subsystems) {
            s.stop();
        }
    }
}