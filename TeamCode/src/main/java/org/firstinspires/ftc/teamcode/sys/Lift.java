package org.firstinspires.ftc.teamcode.sys;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lift {
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor axis;
    public static final PIDFCoefficients AXIS_PIDF = new PIDFCoefficients(0.0032, 0.001, 0.00015, 0);
    public static final PIDFCoefficients LIFT_PIDF = new PIDFCoefficients(0.004, 0.001, 0.0003, 0.00013);

    public static final int AXIS_8_POINT_BASKET = -869;
    public static final int LIFT_8_POINT_BASKET = 2000;

    public static final int LIFT_TRAS = 10;
    public static final int AXIS_TRAS = -755;//
    public static final int AXIS_TRAS_TELEOP = -669;
    public static final int AXIS_AUTO_TRANSFER = -740 + 9;

    public static final int HP_LIFT_UP_POSITION = 1100;
    public static final int HP_LIFT_GET_SPEC = 10;
    public static final int AXIS_GET_SPEC_HP = -1100;
    //    public static final int AXIS_PLACE_SPEC = -650;
//
//    public static final int LIFT_PLACE_SPEC = 1500;
    public static final int AXIS_PLACE_SPEC = -400;

    public static final int LIFT_PLACE_SPEC = 1237;
    public static final int LIFT_FINISH_SPEC = 1200;


    public static final int LIFT_MOVE_UP = 500;
    public static final int LIFT_READY_TO_TRANSFER = 700;
    public static final double AXIS_ZERO_POS = -75;
    public static final double LIFT_ZERO_POS = 110;

    private PIDFController axisController = new PIDFController(AXIS_PIDF.p, AXIS_PIDF.i, AXIS_PIDF.d, AXIS_PIDF.f);
    private PIDFController liftController = new PIDFController(LIFT_PIDF.p, LIFT_PIDF.i, LIFT_PIDF.d, LIFT_PIDF.f);

    public Lift(DcMotor leftLift, DcMotor rightLift, DcMotor axis)
    {
        this.rightLift = rightLift;
        this.leftLift = leftLift;
        this.axis = axis;
    }

    public void MOVE_THE_LIFT_UP()
    {
        liftController.setSetPoint(LIFT_MOVE_UP);
        axisController.setSetPoint(AXIS_TRAS);
    }

    public void BASKET_8_POINT()
    {
        axisController.setSetPoint(AXIS_8_POINT_BASKET);
        liftController.setSetPoint(LIFT_8_POINT_BASKET);
    }
    public void TRANSFER_AUTO()
    {
        setMotorToPos(leftLift, LIFT_TRAS+20, 1);
        setMotorToPos(rightLift, LIFT_TRAS+20, 1);
        setMotorToPos(axis, -620, 1);
    }
    public void BASKET_8_POINT_AUTO()
    {
        setMotorToPos(rightLift, LIFT_8_POINT_BASKET, 1);
        setMotorToPos(leftLift, LIFT_8_POINT_BASKET, 1);
        setMotorToPos(axis, AXIS_8_POINT_BASKET, 1);
    }
    public void BASKET_8_POINT_AUTOSLOW()
    {
        setMotorToPos(rightLift, LIFT_8_POINT_BASKET, 0.8);
        setMotorToPos(leftLift, LIFT_8_POINT_BASKET, 0.8);
        setMotorToPos(axis, AXIS_8_POINT_BASKET, 0.8);
    }

    public void TRANSFER()
    {
        liftController.setSetPoint(LIFT_TRAS);
        axisController.setSetPoint(-777);
    }

    public void SET_MOTOR_TO_ZERO()
    {
        setMotorToPos(axis, -75, 1);
        setMotorToPos(leftLift, 110, 1);
        setMotorToPos(rightLift, 110, 1);
    }
    public void GET_FROM_HP_UP()
    {
        liftController.setSetPoint(HP_LIFT_UP_POSITION);
        axisController.setSetPoint(AXIS_8_POINT_BASKET);
    }

    public void GET_FROM_HP_DOWN()
    {
        liftController.setSetPoint(HP_LIFT_GET_SPEC);
        axisController.setSetPoint(AXIS_GET_SPEC_HP);
    }
    public void GET_FROM_HP_DOWN_AUTO()
    {
        setMotorToPos(leftLift, HP_LIFT_GET_SPEC, 1);
        setMotorToPos(rightLift, HP_LIFT_GET_SPEC, 1);
        setMotorToPos(axis, AXIS_GET_SPEC_HP, 1);
    }

    public int get_lift_position()
    {
        return rightLift.getCurrentPosition();
    }

    public int get_axis_position()
    {
        return axis.getCurrentPosition();
    }

    public void PLACE_SPEC()
    {
        liftController.setSetPoint(LIFT_PLACE_SPEC);
        axisController.setSetPoint(AXIS_PLACE_SPEC);
    }
    public void PLACE_SPEC_AUTO()
    {
        setMotorToPos(leftLift, LIFT_PLACE_SPEC, 1);
        setMotorToPos(rightLift, LIFT_PLACE_SPEC, 1);
        setMotorToPos(axis, AXIS_PLACE_SPEC, 1);
    }

    public void SPEC_FINISH()
    {

        liftController.setSetPoint(LIFT_FINISH_SPEC);
        axisController.setSetPoint(AXIS_PLACE_SPEC);
    }

    public void TRANSFER_FORM_FAR()
    {
        //pass
    }

    public boolean isAxisInPosition()
    {
        return Math.abs(axis.getTargetPosition() - axis.getCurrentPosition()) <= 10;
    }

    public boolean isLiftInPosition()
    {
        boolean leftLiftInRange = Math.abs(leftLift.getTargetPosition() - leftLift.getCurrentPosition()) <= 10;
        boolean rightLiftInRange = Math.abs(rightLift.getTargetPosition() - rightLift.getCurrentPosition()) <= 10;
        return leftLiftInRange && rightLiftInRange;
    }

    public void runAxisToSetpoint() {
        axis.setPower(axisController.calculate(axis.getCurrentPosition()));
    }

    public void setSetpoints(double lift, double axis) {
        liftController.setSetPoint(lift);
        axisController.setSetPoint(axis);
    }

    public void runLiftToSetpoint() {
        double liftPower = liftController.calculate(rightLift.getCurrentPosition());
        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);
    }

    public boolean isInPosition()
    {
        return isLiftInPosition() && isAxisInPosition();
    }

    public void TRANSFER_TELEOP()
    {
        liftController.setSetPoint(LIFT_TRAS);
        axisController.setSetPoint(-760);
    }


    private boolean setMotorToPos(DcMotor motor, int pos, double power)
    {

        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        return true;
    }

    private boolean StopMotorIfReachedPosition(DcMotor motor)
    {

        if (!motor.isBusy() && motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }


        return true;
    }


}
