//package org.firstinspires.ftc.teamcode.drive.auton;
//
//import androidx.annotation.CallSuper;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.drive.vision.BoxDetectionPipeline;
//import org.firstinspires.ftc.teamcode.drive.Robot;
//
//import static org.firstinspires.ftc.teamcode.drive.vision.BoxDetectionPipeline.BoxPosition;
//
///**
// * This serves as a base for all autonomous programs to be built off of. To create a new auton,
// * simply create a new class and extend {@link BaseAuton}. Make sure you override {@link #runOpMode()}
// * and {@link #setRobotPosition()}. When overriding {@link #runOpMode()}, make sure you only
// * include code for stuff after init. Do not include {@link #waitForStart()} in the function.
// */
//public abstract class BaseAuton extends LinearOpMode {
//    public Robot robot;
//    public BoxDetectionPipeline pipeline;
//
//    public BoxPosition boxPosition;
//    public boolean firstTime = true;
//    public boolean runFSM = false;
//
//    public ElapsedTime autonWaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//    public static LeftRed LeftRedState;
//    public static MiddleRed MiddleRedState;
//    public static RightRed RightRedState;
//
//    @Override @CallSuper
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(hardwareMap, telemetry);
//        pipeline = new BoxDetectionPipeline();
//        robot.init();
//        robot.dashboardInit();
//        robot.webcamInit(pipeline);
//
//        setRobotPosition();
//
//        robot.boxUp();
//        robot.updateAll();
//
//        sleep(150); // just to make sure webcam is properly initialized before updateBoxPosition()
//
//        while (!opModeIsActive()) updateBoxPosition();
//
//        waitForStart();
//
//        autonWaitTimer.reset();
//        robot.odoTimer.reset();
//        robot.webcam.stopStreaming();
//    }
//
//    public void updateBoxPosition() {
//        //todo clean this up
//        if (pipeline.position == null) {
//            telemetry.addData("still working on it", "gimme a sec");
//        } else {
//            boxPosition = pipeline.position;
//            telemetry.addData(boxPosition.name() + " position", "Waiting for start");
//        }
//        telemetry.update();
//        sleep(75); //so we don't burn cpu cycles
//    }
//
//    /**
//     * A function for {@link BaseAuton} subclasses to override to give the robot an initial position.
//     * Simply just put {@code robot.setPosition(x, y, theta)} in the function signature.
//     * @see Robot#setPosition(double, double, double)
//     */
//    public abstract void setRobotPosition();
//
//}
