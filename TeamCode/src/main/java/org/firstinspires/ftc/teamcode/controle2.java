package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;




import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "PRINCIPaL")

public class controle2 extends LinearOpMode {

    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M4;
    DcMotor M5;
    BHI260IMU imu;
    Servo Sr1;
    Servo Sr2;
    Servo Sr3;
    Servo Sr4;

    public Runnable t1 = new Thread(new Runnable() {
        @Override
        public void run() {
            M3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (gamepad2.left_stick_y == 0){
                int encouder3 = M3.getCurrentPosition();
                while (gamepad2.left_stick_y == 0) {
                    int encouder3A = M3.getCurrentPosition();
                    int P3D = encouder3 - encouder3A;
                    double K3 = P3D * 0.02;
                    M3.setPower(K3);
                    if (gamepad2.left_stick_y != 0){
                        telemetry.addData("bom dia", "Bom dia");
                        break;
                    }
                }
            }
        }
    });

    @Override
    public void runOpMode() {
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");
        M5 = hardwareMap.get(DcMotor.class, "M5");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        Sr1 = hardwareMap.get(Servo.class, "Sr1");
        Sr2 = hardwareMap.get(Servo.class, "Sr2");
        Sr3 = hardwareMap.get(Servo.class, "Sr3");
        Sr4 = hardwareMap.get(Servo.class, "Sr4");
        //Teste do IMU
        IMU.Parameters myIMUparametros;
        myIMUparametros = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        myIMUparametros = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                0,
                                0,
                                -90,
                                0
                        )
                )
        );

        imu.initialize(myIMUparametros);

        waitForStart();

        YawPitchRollAngles robotOrientation;
        //Sr4.setPosition(0);
        Sr4.setPosition(0.5);
        if (opModeIsActive()) {

            double posicao = 0;
            new Thread(t1).start();
            boolean pulso = true;
            boolean PinsaL = true;
            boolean PinsaR = false;
            Sr4.setPosition(0.5);

            while (opModeIsActive()) {
                double leftStickY = gamepad1.left_stick_y ;
                double RightSticky = -this.gamepad1.right_stick_y;
                double RT = gamepad1.right_trigger;
                double LT = gamepad1.left_trigger;
                double RT2 = -this.gamepad2.right_stick_y;
                boolean controle = true;

                if (gamepad2.right_stick_button && gamepad2.left_stick_button){
                    M4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (gamepad1.left_bumper){
                    Sr4.setPosition(1);
                }

                if (M4.getCurrentPosition() >= 1167){
                    M4.setPower(0);
                    controle = false;
                    if (gamepad2.left_trigger > 0 && M4.getCurrentPosition() > 1167) {
                        controle = true;
                        M4.setPower(-this.gamepad2.right_stick_y);
                    }
                }

                //-------------------------------------------------
                if (gamepad2.a ) {
                    while (gamepad2.a){
                    }
                    if (pulso){
                        pulso = false;
                    }
                    else {
                        pulso = true;
                    }
                }

                if (pulso){
                    Sr1.setPosition(0.05);
                }
                if (!pulso){
                    Sr1.setPosition(1);
                }
                //-------------------------------------------------
                if (gamepad2.left_bumper) {
                    while (gamepad2.left_bumper){
                    }
                    if (PinsaL){
                        PinsaL = false;
                    }
                    else {
                        PinsaL = true;
                    }
                }
                if (gamepad2.right_bumper) {
                    while (gamepad2.right_bumper){
                    }
                    if (PinsaR){
                        PinsaR = false;
                    }
                    else {
                        PinsaR = true;
                    }
                }
                if (gamepad2.b) {
                    while (gamepad2.b){
                    }
                    if (PinsaR){
                        PinsaR = false;
                    }
                    else {
                        PinsaR = true;
                    }
                    if (PinsaL){
                        PinsaL = false;
                    }
                    else {
                        PinsaL = true;
                    }

                }

                if (PinsaL){
                    Sr2.setPosition(1);
                }
                if (!PinsaL){
                    Sr2.setPosition(-1);
                }
                if (PinsaR){
                    Sr3.setPosition(1);
                }
                if (!PinsaR){
                    Sr3.setPosition(-1);
                }

                M1.setPower(leftStickY);
                M2.setPower(RightSticky);

                if (gamepad1.right_bumper){
                    int encouder1 = M1.getCurrentPosition();
                    int encouder2 = M2.getCurrentPosition();
                    while (gamepad1.right_bumper) {
                        int encouder1A = M1.getCurrentPosition();
                        int encouder2A = M2.getCurrentPosition();

                        int P2D = encouder2 - encouder2A;
                        int P1D = encouder1 - encouder1A;
                        double K = P2D * 0.02;
                        double K1 = P1D * 0.02;
                        M2.setPower(K);
                        M1.setPower(K1);
                        telemetry.addData("encouder", M2.getCurrentPosition());
                        telemetry.addData("K", K);
                        telemetry.update();
                    }

                }
                M3.setPower(gamepad2.left_stick_y * 0.5);

                if (controle) {
                    if (gamepad2.right_trigger > 0) {
                        M4.setPower(gamepad2.right_trigger);
                    }
                    else if (gamepad2.left_trigger > 0) {
                        M4.setPower(-this.gamepad2.left_trigger);
                    }
                    else {
                        M4.setPower(0);
                    }
                }
                //M5.setPower(gamepad1.left_stick_x);

                if (gamepad1.left_trigger > 0) {
                    M5.setPower(gamepad1.left_trigger);
                }
                else if (gamepad1.right_trigger > 0) {
                    M5.setPower(-this.gamepad1.right_trigger);
                }
                else {
                    M5.setPower(0);
                }



                //Variaveis IMU
                robotOrientation = imu.getRobotYawPitchRollAngles();
                double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
                double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);

                //Variaveis Encouder
                int Dencouder = M2.getCurrentPosition();


                telemetry.addData("Yaw: ", Yaw);
                telemetry.addData("Pitch: ", Pitch);
                telemetry.addData("Roll: ", Roll);
                telemetry.addData("esquerdo: ", leftStickY);
                telemetry.addData("sas", gamepad2.right_stick_y);
                telemetry.addData("encouder", M2.getCurrentPosition());
                telemetry.addData("posicao", pulso);
                telemetry.addData("Servo Position", Sr1.getPosition());
                telemetry.addData("Encuder garra", M4.getCurrentPosition());
                telemetry.update();
            }

        }

    }

}
