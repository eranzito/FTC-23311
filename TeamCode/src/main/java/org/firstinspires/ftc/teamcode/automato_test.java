package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Saida Quadro")
public class automato_test extends LinearOpMode {
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M5;
    Servo Sr1;
    Servo Sr2;
    Servo Sr3;

    int encouder;
    int encouder1A;
    int encouder2A;
    int encouder5;

    @Override
    public void runOpMode() throws InterruptedException {
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M5 = hardwareMap.get(DcMotor.class, "M5");
        Sr1 = hardwareMap.get(Servo.class, "Sr1");
        Sr2 = hardwareMap.get(Servo.class, "Sr2");
        Sr3 = hardwareMap.get(Servo.class, "Sr3");
        M1.setDirection(DcMotorSimple.Direction.REVERSE);
        //  .......................................................//

        Sr2.setPosition(-1);
        Sr3.setPosition(1);
        sleep(1500);
        Sr1.setPosition(1);

        int encouder3 = M3.getCurrentPosition();
        while (true) {
            int encouder3A = M3.getCurrentPosition();
            int P3D = encouder3 - encouder3A;
            double K3 = P3D * 0.02;
            M3.setPower(K3);
            if (opModeIsActive()){
                telemetry.addData("bom dia", "Bom dia");
                break;
            }
        }

        waitForStart();
        if (opModeIsActive()) {

            M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(M3.getCurrentPosition() > -50) {
                M3.setPower(-0.7);
            }
            M3.setPower(0);
            //linear(5, true);
            lados(false, 300);
            //virar(true, 440);
            linear(60, true);
            sleep(1000);
            while (opModeIsActive()) {
                Sr1.setPosition(0.05);
                sleep(1000);
                Sr2.setPosition(1);
                Sr3.setPosition(-1);
                telemetry.addData("M3", M3.getCurrentPosition());
                telemetry.update();
            }

        }
    }


    public void linear(int cm, boolean frente) {
        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //encouder = cm * 17;


        if (frente){
            encouder = cm * 14;
            while(encouder1A <= encouder && encouder2A <= encouder) {
                encouder1A = M1.getCurrentPosition();
                encouder2A = M2.getCurrentPosition();
                M1.setPower(0.5);
                M2.setPower(0.5);
            }
        } else {
            encouder = cm * -14;
            while(encouder1A >= encouder && encouder2A >= encouder) {
                encouder1A = M1.getCurrentPosition();
                encouder2A = M2.getCurrentPosition();
                M1.setPower(-0.5);
                M2.setPower(-0.5);
            }

        }
        M1.setPower(0);
        M2.setPower(0);
    }


    public void virar( boolean direction, int encouderC){
        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction){
            while(encouder1A <= encouderC && encouder2A >= -encouderC){
                encouder1A = M1.getCurrentPosition();
                encouder2A = M2.getCurrentPosition();
                M1.setPower(0.7);
                M2.setPower(-0.7);
            }

        } else  {
            while(encouder1A >= -encouderC && encouder2A <= encouderC){
                encouder1A = M1.getCurrentPosition();
                encouder2A = M2.getCurrentPosition();
                M1.setPower(-0.7);
                M2.setPower(0.7);

            }
        }
        M1.setPower(0);
        M2.setPower(0);

    }

    public void lados (boolean direction, int encouderM){
        M5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction){
            while(encouder5 <= encouderM){
                encouder5 = M5.getCurrentPosition();
                M5.setPower(1.0);
            }

        } else  {
            while(encouder5 >= -encouderM){
                encouder5 = M5.getCurrentPosition();
                M5.setPower(-1.0);
            }
        }

        M5.setPower(0);
        M5.setPower(0);

    }

}