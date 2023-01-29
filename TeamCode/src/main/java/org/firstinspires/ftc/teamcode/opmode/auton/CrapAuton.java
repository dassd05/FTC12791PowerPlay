//package org.firstinspires.ftc.teamcode.opmode.auton;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//public class CrapAuton extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Robot robot = new Robot(hardwareMap, telemetry);
//        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
//        robot.initWebcam(pipeline);
//
//        SignalDetectionPipeline.ParkPosition parkPosition;
//        sleep(250);
//
//        while (opModeInInit()) {
//            sleep(100);
//            parkPosition = pipeline.position;
//            telemetry.addData("position", parkPosition);
//            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
//            telemetry.update();
//        }
//
//        TrajectorySequence startToCycle = robot.butterfly.trajectorySequenceBuilder(robot.getPosition())
//                .forward(0)
//                .turn(0)
//                .build();
//        TrajectorySequence afterCycle = robot.butterfly.trajectorySequenceBuilder(startToCycle.end())
//                .lineToLinearHeading(new Pose2d())
//                .strafeTo(new Vector2d())
//                .build();
//
//        waitForStart();
//        resetRuntime();
//
//        if (opModeIsActive()) {
//            robot.butterfly.followTrajectorySequence(startToCycle);
//            robot.intakeOuttake.deploy(4, 3);
//            sleep(500);
//            robot.intakeOuttake.outtake();
//            sleep(250);
//            robot.intakeOuttake.stoptake();
//        }
//        int i = 0;
//        while (opModeIsActive() && getRuntime() < 25 && ++i < 5) {
//            //cycle
//            robot.intakeOuttake.setTargetAbsolute(0, 0, 0);
//            robot.intakeOuttake.intake();
//            sleep(500);
//            robot.intakeOuttake.deploy(4, 3);
//            sleep(500);
//            robot.intakeOuttake.outtake();
//            sleep(250);
//            robot.intakeOuttake.stoptake();
//        }
//        if (opModeIsActive()) {
//            robot.intakeOuttake.collapse();
//            robot.butterfly.followTrajectorySequence(afterCycle);
//        }
//    }
//}
