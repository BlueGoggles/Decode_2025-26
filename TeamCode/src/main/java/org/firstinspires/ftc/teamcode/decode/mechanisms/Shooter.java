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
import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class Shooter {
    private DcMotorEx leftWheel;
    private DcMotorEx rightWheel;

    public Shooter(HardwareMap hardwareMap) {
        VoltageSensor batteryVoltageSensor;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0.5, 5, 12);

        leftWheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
//        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        setPIDFCoefficients(leftWheel, MOTOR_VELO_PID, batteryVoltageSensor);

        rightWheel = hardwareMap.get(DcMotorEx.class, "rightShooter");
//        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        setPIDFCoefficients(rightWheel, MOTOR_VELO_PID, batteryVoltageSensor);
    }

    public class StartShooter implements Action {

        final private double power;

        public StartShooter(double power) {
            if (power == 0) {
                power = Constants.DEFAULT_SHOOTER_VELOCITY;
            }
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

            leftWheel.setVelocity(this.power);
            rightWheel.setVelocity(this.power);

            return false;
        }
    }

    public Action startShooter(double power) {
        return new StartShooter(power);
    }

    public Action startShooter() {
        return new StartShooter(leftWheel.getVelocity());
    }

    public class StopShooter implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWheel.setVelocity(0);
            rightWheel.setVelocity(0);
            return false;
        }
    }

    public Action stopShooter() {
        return new StopShooter();
    }

    public class AdjustShooterSpeed implements Action {

        boolean increaseFlag;

        public AdjustShooterSpeed(boolean increaseFlag) {
            this.increaseFlag = increaseFlag;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

            double currentSpeed = leftWheel.getVelocity();
            if(this.increaseFlag) {
                leftWheel.setVelocity(currentSpeed + Constants.CHANGE_SHOOTER_VELOCITY_BY);
                rightWheel.setVelocity(currentSpeed + Constants.CHANGE_SHOOTER_VELOCITY_BY);
            } else {
                leftWheel.setVelocity(currentSpeed - Constants.CHANGE_SHOOTER_VELOCITY_BY);
                rightWheel.setVelocity(currentSpeed - Constants.CHANGE_SHOOTER_VELOCITY_BY);
            }
            return false;
        }
    }

    public Action adjustShooterSpeed(boolean increaseFlag) {
        return new AdjustShooterSpeed(increaseFlag);
    }

    public class ReverseShooter implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

            leftWheel.setVelocity(Constants.DEFAULT_SHOOTER_VELOCITY);
            rightWheel.setVelocity(Constants.DEFAULT_SHOOTER_VELOCITY);

            return false;
        }
    }

    public Action reverseShooter() {
        return new ReverseShooter();
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
