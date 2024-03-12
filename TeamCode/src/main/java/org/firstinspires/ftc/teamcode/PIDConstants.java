package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class PIDConstants {
    public static double Kp = 1.2; //2;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.1;   // Derivative gain
}