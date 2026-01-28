package org.firstinspires.ftc.teamcode.DECODE;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.Constantes.FConstants;
import org.firstinspires.ftc.teamcode.Constantes.LConstants;

@Autonomous(name = "VermelhoMenor")
public class AutoVermelhoMenor extends LinearOpMode {
    Follower follower;
    DcMotor motorColetor, motorColetor2, lancador;

    int etapaAtual = 0;
    double voltagem;

    Pose startPose           =   new Pose(80, 10,Math.toRadians(90));
    Pose poseDeLancamento    =   new Pose(86,20);
    Pose poseFinal           =   new Pose(96,30,Math.toRadians(90));

    Pose coleta21Final       =   new Pose(119,34);
    Pose coleta21Ctrl        =   new Pose(86,35);

    @Override
    public void runOpMode() throws InterruptedException {
        boolean truE = true;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorColetor  = hardwareMap.dcMotor.get("motorCO");
        motorColetor2 = hardwareMap.dcMotor.get("motorCO2");
        lancador      = hardwareMap.dcMotor.get("motorLancador");
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PathChain path21_1  = new PathBuilder()
                .addPath(criarCurva(poseDeLancamento,coleta21Ctrl,coleta21Final))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        PathChain path21_2  = new PathBuilder()
                .addPath(criarCurva(coleta21Final, coleta21Ctrl, poseDeLancamento))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(66))
                .build();
        PathChain pathStart = new PathBuilder()
                .addPath(criarLinha(startPose,poseDeLancamento))
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(66))
                .build();
        PathChain pathFinal = new PathBuilder()
                .addPath(criarLinha(poseDeLancamento,poseFinal))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();



        waitForStart();
        while (opModeIsActive() && truE){
            follower.update();
            salvarAlianca("vermelho");
            salvarPosicaoDoRobo(follower);
            voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();
            switch (etapaAtual){
                case 0:
                    follower.followPath(pathStart);
                    if(follower.atParametricEnd()){
                        lancarArtefato(calcularPotencia());
                        etapaAtual++;
                    }
                    break;
                case 1:
                    follower.followPath(path21_1);
                    if(follower.getPose().getY() > 27){
                        coletar(false);
                    }
                    if(follower.atParametricEnd()){
                        coletar(true);
                        etapaAtual++;
                    }
                    break;
                case 2:
                    follower.followPath(path21_2);
                    if(follower.getPose().getX() < 97){
                        lancador.setPower(calcularPotencia());
                    }
                    if(follower.atParametricEnd()){
                        lancarArtefato(calcularPotencia());
                        etapaAtual++;
                    }
                    break;
                case 3:
                    follower.followPath(pathFinal);
                    if(follower.atParametricEnd()){
                        etapaAtual = -1;
                        truE = false;
                    }
                    break;
                default:
                    break;
            }
        }

    }
    void lancarArtefato(double potencia){
        motorColetor2.setPower(.8);
        motorColetor.setPower(.8);
        sleep(500);
        motorColetor.setPower(0);
        motorColetor2.setPower(0);
        sleep(1000);
        motorColetor.setPower(.8);
        motorColetor2.setPower(.8);
        sleep(1200);
        motorColetor.setPower(0);
        motorColetor2.setPower(0);
        lancador.setPower(0);
    }
    void coletar(boolean parar) {
        if(!parar){
            motorColetor.setPower(1);
            motorColetor2.setPower(1);
        }
        else {
            motorColetor.setPower(0);
            motorColetor2.setPower(-0.2);
            sleep(500);
            motorColetor2.setPower(0);
        }
    }
    Path criarCurva(Pose startPose,Pose ctrlPose, Pose finalPose){
        return new Path(
                new BezierCurve(
                        startPose,ctrlPose,finalPose
                )
        );
    }
    Path criarLinha(Pose startPose,Pose finalPose){
        return new Path(
                new BezierLine(
                        startPose,finalPose
                )
        );
    }
    void salvarPosicaoDoRobo(@NonNull Follower follower){
        double xAgora = follower.getPose().getX();
        double yAgora = follower.getPose().getY();
        double hAgora = follower.getPose().getHeading();

        String fileName = "current_robot_position.cvc";

        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);
        String dataToSave = xAgora + "," + yAgora + "," + hAgora;

        try{
            FileWriter writer = new FileWriter(file);
            writer.write(dataToSave);
            writer.close();

            telemetry.addData("Pose a ser salva: ", "X: %.2f , Y: %.2f , Heading: %.2f",xAgora,yAgora,hAgora);
        } catch (IOException e){
            telemetry.addData("Não foi possível salvar a pose do robô","ERROR:" + e.getMessage());
        }
    }
    double calcularPotencia(){
        return 0.9*(13.5/voltagem);
    }
    void salvarAlianca(String corDaAlianca){
        corDaAlianca = corDaAlianca.toLowerCase();
        String fileName = "cor_da_alianca.cvc";

        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);

        try{
            FileWriter writer = new FileWriter(file);
            writer.write(corDaAlianca);
            writer.close();

            telemetry.addData("Aliança a ser salva: ",corDaAlianca);
        } catch (IOException e){
            telemetry.addData("Não foi possível salvar a  aliança", "ERROR:" + e.getMessage());
        }
    }
}
