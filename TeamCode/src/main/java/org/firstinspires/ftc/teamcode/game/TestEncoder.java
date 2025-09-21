package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestEncoder", group = "Linear Opmode")
public class TestEncoder extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lfd;
    private DcMotor rfd;
    private DcMotor rbd;
    private DcMotor lbd;

    @Override
    public void runOpMode() throws InterruptedException {
        lfd = hardwareMap.get(DcMotor.class, "frontLeft");
        rfd = hardwareMap.get(DcMotor.class, "frontRight");
        rbd = hardwareMap.get(DcMotor.class, "backRight");
        lbd = hardwareMap.get(DcMotor.class, "backLeft");

        lfd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.FORWARD);
        lbd.setDirection(DcMotor.Direction.REVERSE);

        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(DRIVE_SPEED,
                    10000,
                    10000,
                    10000,
                    10000,
                    10000);
        }
    }

    private void encoderDrive(double driveSpeed, double leftFrontInches, double rightFrontInches, double rightBackInches, double leftBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            newLeftFrontTarget = lfd.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rfd.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = lbd.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rbd.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            lfd.setTargetPosition(newLeftFrontTarget);
            rfd.setTargetPosition(newRightFrontTarget);
            lbd.setTargetPosition(newLeftBackTarget);
            rbd.setTargetPosition(newRightBackTarget);

            lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            lfd.setPower(Math.abs(driveSpeed));
            rfd.setPower(Math.abs(driveSpeed));
            lbd.setPower(Math.abs(driveSpeed));
            rbd.setPower(Math.abs(driveSpeed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lfd.isBusy() && rfd.isBusy() && lbd.isBusy() && rbd.isBusy())) {

                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        lfd.getCurrentPosition(), rfd.getCurrentPosition(), lbd.getCurrentPosition(), rbd.getCurrentPosition());
                telemetry.update();
                if (lfd.getCurrentPosition() != rfd.getCurrentPosition() ||
                        lfd.getCurrentPosition() != rbd.getCurrentPosition() ||
                        lfd.getCurrentPosition() != lbd.getCurrentPosition()) {
                    telemetry.addData("Motors equal:", "false");
                } else {
                    telemetry.addData("Motors equal:", "true");
                }
            }

            lfd.setPower(0);
            rfd.setPower(0);
            lbd.setPower(0);
            rbd.setPower(0);

            lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
