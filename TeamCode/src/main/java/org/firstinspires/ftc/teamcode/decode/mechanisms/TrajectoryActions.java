package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

     public Action getTrajectory_1_1(MecanumDrive drive, boolean initialPoseFlag){

         Pose2d pose;

         if (initialPoseFlag) {
             pose = new Pose2d(0, 0, Math.toRadians(0));
         } else {
             pose = drive.localizer.getPose();
         }

         TrajectoryActionBuilder builder = drive.actionBuilder(pose)
//                 .strafeToLinearHeading(new Vector2d(3, 69), Math.toRadians(-52));
                 .strafeToLinearHeading(new Vector2d(3, 69), Math.toRadians(142));

         return builder.build();
     }

    public Action getTrajectory_1_2(MecanumDrive drive){

        TrajectoryActionBuilder builder = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(2, 58), Math.toRadians(180));

        return builder.build();
    }

}
