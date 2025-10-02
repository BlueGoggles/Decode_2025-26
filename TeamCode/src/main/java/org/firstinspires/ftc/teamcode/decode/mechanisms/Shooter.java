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
        leftWheel = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rightWheel = hardwareMap.get(DcMotorEx.class, "topShooter");
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class StartShooter implements Action {
//        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
                leftWheel.setPower(0.45);
                rightWheel.setPower(0.45);
//                initialized = true;
//            }

            return false;
        }
    }

    public Action startShooter() {
        return new StartShooter();
    }

    public class StopShooter implements Action {
//        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
//                initialized = true;
//            }

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
