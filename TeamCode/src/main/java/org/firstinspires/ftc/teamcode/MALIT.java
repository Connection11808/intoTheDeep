/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sys.IntakeArm;



/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Config
class TestPID {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;
    public static double setpoint = 0;
}

@TeleOp(name = "encoderTEST_DONT_RUN", group = "Linear OpMode")
//@Disabled
//@Disabled
public class MALIT extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor motor = null;
    private  DcMotor leftLift = null;
    private  DcMotor rightLift = null;
    private  DcMotor retriveal = null;
    private PIDFController pidfController;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode()
    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        motor = hardwareMap.get(DcMotor.class, "axis");
        retriveal = hardwareMap.get(DcMotor.class, "retriveal");
        retriveal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retriveal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        pidfController = new PIDFController(TestPID.kP, TestPID.kI, TestPID.kD, TestPID.kF);
        pidfController.setTolerance(25);



        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        double setpoint = 0;
        boolean isRunning = false;
        while (opModeIsActive())
        {

            if(gamepad1.a){
                setMotorToPos(retriveal,-100,1);
            }

//            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            double POWER = gamepad1.left_stick_x;
//
//            if (gamepad1.a)
//            {
//                setpoint = TestPID.setpoint;
//            }
//
//            pidfController.setPIDF(TestPID.kP, TestPID.kI, TestPID.kD, TestPID.kF);
//            pidfController.setSetPoint(setpoint);
//            double power = pidfController.calculate(motor.getCurrentPosition());
////            rightLift.setPower(power);
////            leftLift.setPower(power);
//            motor.setPower(power);
//
////            if (gamepad1.b)
////            {
////                setMotorToPos(-900, 1);
////                isRunning = true;
////            }
////
////            else if (gamepad1.x)
////            {
////                setMotorToPos(0, 1);
////                isRunning = true;
////            }
////
////            if (isRunning)
////            {
////                StopMotorIfReachedPosition();
////           } else
////            {
////                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////                motor.setPower(POWER);
////            }
////
////            if (POWER != 0)
////            {
////                isRunning = false;
////            }


//            telemetry.addData("leftpos", leftLift.getCurrentPosition());
//            telemetry.addData("rightpos", rightLift.getCurrentPosition());
//            telemetry.update();
//            FtcDashboard.getInstance().getTelemetry().addData("leftpos", leftLift.getCurrentPosition());
//            FtcDashboard.getInstance().getTelemetry().addData("rightpos", rightLift.getCurrentPosition());
//            FtcDashboard.getInstance().getTelemetry().addData("left power", leftLift.getPower());
//            FtcDashboard.getInstance().getTelemetry().addData("left power", rightLift.getPower());
//            FtcDashboard.getInstance().getTelemetry().addData("motor power", motor.getPower());
//            FtcDashboard.getInstance().getTelemetry().addData("motor pos", motor.getCurrentPosition());
//            FtcDashboard.getInstance().getTelemetry().addData("kP", TestPID.kP);
//            FtcDashboard.getInstance().getTelemetry().addData("kI", TestPID.kI);
//            FtcDashboard.getInstance().getTelemetry().addData("kD", TestPID.kD);
//            FtcDashboard.getInstance().getTelemetry().addData("kF", TestPID.kF);
//            FtcDashboard.getInstance().getTelemetry().addData("setPoint", setpoint);
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }

//    private boolean setMotorToPos(int pos, double power)
//    {
//        motor.setTargetPosition(pos);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setPower(-power);
//        return true;
//    }

    private boolean StopMotorIfReachedPosition()
    {

        if (!motor.isBusy())
        {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        telemetry.addData("motorpos", motor.getCurrentPosition());
        telemetry.update();


        return true;
    }
    private boolean setMotorToPos(DcMotor motor, int pos, double power)
    {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        return true;
    }
}
