package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotorEx leftShooter = null;
    private DcMotorEx rightShooter = null;
    private OpenCvCamera controlHubCam = null; // Use OpenCvCamera class from FTC SDK

    private AprilTagProcessor aprilTag = null; // Used for managing the AprilTag detection process.
    private VisionPortal visionPortal = null; // Used to manage the video source.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void initialize()    {

     // Expansion hub motors
        leftShooter  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftShooter" );
        rightShooter = myOpMode.hardwareMap.get(DcMotorEx.class, "rightShooter");

        getLeftShooter().setDirection(DcMotorEx.Direction.REVERSE);
        getRightShooter().setDirection(DcMotorEx.Direction.FORWARD);

        setMotorPowers(Constants.ZERO_POWER);
    }

    public void setMotorPowers(double motorPowers) {
        this.getLeftShooter().setPower(motorPowers);
        this.getRightShooter().setPower(motorPowers);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        getLeftShooter().setMode(mode);
        getRightShooter().setMode(mode);
    }

    public void initializeOpenCV(OpenCvPipeline pipeline) {

        // Create an instance of the camera
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        this.controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA), cameraMonitorViewId);

        this.controlHubCam.setPipeline(pipeline);

        this.controlHubCam.openCameraDevice();
        this.controlHubCam.startStreaming(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    public void releaseResourcesForOpenCV() {

        getControlHubCam().stopRecordingPipeline();
        getControlHubCam().stopStreaming();
        getControlHubCam().closeCameraDevice();
    }

    public void initializeAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA))
                .addProcessor(aprilTag)
                .build();
    }

    public LinearOpMode getMyOpMode() {
        return this.myOpMode;
    }

    public DcMotorEx getLeftShooter() {
        return this.leftShooter;
    }

    public DcMotorEx getRightShooter() {
        return this.rightShooter;
    }

    public OpenCvCamera getControlHubCam() {
        return this.controlHubCam;
    }

    public AprilTagProcessor getAprilTag() {
        return this.aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return this.visionPortal;
    }
}
