package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter2 {
    private DcMotorEx shooter;

    public Shooter2(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class StartShooter2 implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shooter.setPower(1.0);
            return false;
        }
    }

    public Action startShooter2() {
        return new StartShooter2();
    }

    public class StopShooter2 implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shooter.setPower(0);
            return false;
        }
    }

    public Action stopShooter2() {
        return new StopShooter2();
    }
}
