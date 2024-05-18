package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PIDFControl {

    private Telemetry telemetry;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastVelocity = 0;

    public static double Kp = 0, Ki = 0, Kd = 0, Kg = 0;

    public PIDFControl(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public double calculatePIDF(double reference, double state) {
        double error = reference - state;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;

        elapsedTime.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + Kg;

        return output;
    }

}
