package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public double integralRange;
    public double integralSum;

    public double setpoint = 0;

    private final ElapsedTime timer;

    public PIDFController(double kP, double kI, double kD, double kF, double integralRange)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralRange = integralRange;
        this.integralSum = 0;

        this.setSetpoint(0);

        this.timer = new ElapsedTime();
    }

    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
    }

    public double compute(double currentValue)
    {
        double deltaTime = timer.seconds();
        double error = currentValue - setpoint;

        if (Math.abs(error) < integralRange)
        {
            integralSum = 0;
        }

        double derivative = error / deltaTime;
        integralSum += error * deltaTime;
        double result = (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
        timer.reset();

        return result;
    }

}
