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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Shooter {
    private DcMotorEx leftWheel;
    private DcMotorEx rightWheel;

    public Shooter(HardwareMap hardwareMap) {
        leftWheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        rightWheel = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class StartShooter implements Action {

        final private double power;

        public StartShooter(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWheel.setPower(this.power);
            rightWheel.setPower(this.power);
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
}
