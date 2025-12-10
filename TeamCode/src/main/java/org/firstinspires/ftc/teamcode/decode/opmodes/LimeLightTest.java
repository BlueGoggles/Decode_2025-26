package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerStopperServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "LimeLight Test", group = "DecodeAutonomous")
public class LimeLightTest extends LinearOpMode {

    private Limelight3A limelight3A;

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltMotor intakeBeltMotor = new IntakeBeltMotor(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
//        KickerStopperServo kickerStopperServo = new KickerStopperServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);// april tag 12 pipeline

        limelight3A.start();

        waitForStart();

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
        }
    }
}