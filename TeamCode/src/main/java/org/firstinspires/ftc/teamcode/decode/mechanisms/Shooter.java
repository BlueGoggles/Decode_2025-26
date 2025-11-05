package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Shooter {
    private DcMotorEx leftWheel;
    private DcMotorEx rightWheel;

    public Shooter(HardwareMap hardwareMap) {
        VoltageSensor batteryVoltageSensor;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0.5, 5, 12);

        leftWheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        setPIDFCoefficients(leftWheel, MOTOR_VELO_PID, batteryVoltageSensor);

        rightWheel = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        setPIDFCoefficients(rightWheel, MOTOR_VELO_PID, batteryVoltageSensor);
    }

    public class StartShooter implements Action {

        final private double power;

        public StartShooter(double power) {
            this.power = power;
        }

        @Override
        public synchronized boolean run(@NonNull TelemetryPacket packet) {

            leftWheel.setPower(this.power);
            rightWheel.setPower(this.power);

//            try {
//                wait(3000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            return false;
        }
    }

    public Action startShooter(double power) {
        return new StartShooter(power);
    }

    public class StopShooter implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            return false;
        }
    }

    public Action stopShooter() {
        return new StopShooter();
    }

    public DcMotorEx getLeftWheel() {
        return leftWheel;
    }

    public DcMotorEx getRightWheel() {
        return rightWheel;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients, VoltageSensor batteryVoltageSensor) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}
