package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

public class BulkReading {

    public static int pSlideMotor;
    public static double vSlideMotor;

    private JVBoysSoccerRobot robot;
    public BulkReading(JVBoysSoccerRobot robot) {
        this.robot = robot;
    }

    public void readAll() {
        pSlideMotor = robot.slideMotor.getCurrentPosition();
        vSlideMotor = robot.slideMotor.getVelocity();
    }

}
