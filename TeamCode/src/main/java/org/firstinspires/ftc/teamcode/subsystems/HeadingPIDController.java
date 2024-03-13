package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.PIDConstants;

public class HeadingPIDController {
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;
    double lastError = 0;
    double integralSum;
    double lastTimestamp = (double) System.currentTimeMillis() / 1000;

    double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public double PIDControl(double reference, double state) {
        integralSum = 0;
        double error = angleWrap(reference - state);
        integralSum += error * (((double) System.currentTimeMillis() / 1000) - lastTimestamp);
        double derivative = (error - lastError) / (((double) System.currentTimeMillis() / 1000) - lastTimestamp);
        lastError = error;
        lastTimestamp = (double) System.currentTimeMillis() / 1000;
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    public HeadingPIDController(){
    }
}
