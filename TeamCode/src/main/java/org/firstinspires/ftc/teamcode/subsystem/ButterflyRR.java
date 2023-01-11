package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.Arrays;
import java.util.function.Supplier;


//@Config
/*
HOW ROADRUNNER SAMPLE WORKS:
You have your drive class.
It has a custom TrajectoryFollower and TrajectorySequenceRunner, their parameters being various PIDCoefficients and tolerance
The follower is specific to unique drivetrains, whereas the runner is general.
Your class produces a TrajectoryBuilder built with your specified constraints for the end user to build Trajectories with.
The user tells your class to follow each given Trajectory, and each time you update(), you gather your current pose and velocity,
and use the runner to calculate what power you want your motors to run at.
Pose can either be calculated using your drivetrain's kinematics, or using an external localizer.
You also need your drivetrain's kinematics to convert the runner's provided DriveSignal to motor powers.

Roadrunner's provided sample drives are very bloated, so I had to summarize and strip down to the essence to make sense of what was going on.
Now, todo put roadrunner stuff in this ButterflyRR child class, or keep it all in the parent Butterfly class?
 */
public class ButterflyRR extends Butterfly {
    // todo
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PIDM = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PIDT = new PIDCoefficients(0, 0, 0);

    private static final TrajectoryVelocityConstraint VELOCITY_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCELERATION_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);


    private final TrajectorySequenceRunner mecanumTrajectoryRunner;
    private final TrajectorySequenceRunner tankTrajectoryRunner;

    public ButterflyRR(HardwareMap hardwareMap, Supplier<Pose2d> position, Supplier<Pose2d> velocity) {
        super(hardwareMap, position, velocity);

        // todo
        TrajectoryFollower followerM = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PIDM,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        TrajectoryFollower followerT = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        mecanumTrajectoryRunner = new TrajectorySequenceRunner(followerM, HEADING_PIDM);
        tankTrajectoryRunner = new TrajectorySequenceRunner(followerT, HEADING_PIDT);
    }

    public ButterflyRR(HardwareMap hardwareMap, Supplier<Pose2d> position) {
        this(hardwareMap, position, () -> null);
    }


    protected TrajectorySequenceRunner getTrajectoryRunner() {
        if (getState() == State.MECANUM) return mecanumTrajectoryRunner;
        if (getState() == State.TRACTION) return tankTrajectoryRunner;
        else return null;
    }


    // todo cleanup
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        getTrajectoryRunner().followTrajectorySequenceAsync(
                trajectorySequenceBuilder(position.get())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        getTrajectoryRunner().followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        getTrajectoryRunner().followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return getTrajectoryRunner().getLastPoseError();
    }

    @Override
    public void update() {
        DriveSignal signal = getTrajectoryRunner().update(position.get(), velocity.get());
        if (signal != null) {
            if (getState() == State.MECANUM) setMotorPowers(mecanum.setDriveSignal(signal));
            else if (getState() == State.TRACTION) setMotorPowers(tank.setDriveSignal(signal));
        }
        if (getState() == State.STANDSTILL) brake();
        super.update();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) update();
    }

    public boolean isBusy() {
        return getTrajectoryRunner().isBusy();
    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
