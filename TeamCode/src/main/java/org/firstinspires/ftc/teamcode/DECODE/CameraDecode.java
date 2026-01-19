package org.firstinspires.ftc.teamcode.DECODE;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class CameraDecode {
    private AprilTagDetection detection;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final ColorBlobLocatorProcessor processorPurple;
    private final ColorBlobLocatorProcessor processorGreen;


    Position cameraPosition = new Position(DistanceUnit.CM,
            0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    public CameraDecode(HardwareMap hardwareMap){
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition,cameraOrientation)
                .setOutputUnits(DistanceUnit.CM,AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();
        processorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,1,1,-1))
                .setBlurSize(5)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();
        processorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,1,1,-1))
                .setBlurSize(5)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTag, processorPurple, processorGreen)
                .setCameraResolution(new Size(640,480))
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .enableLiveView(true)
                .build();
    }
    AprilTagDetection getDetection(){
        for(AprilTagDetection detection : aprilTag.getDetections()){
            if(detection.metadata != null){
                this.detection = detection;
            }
        }
        return detection;
    }
    void resume(){
        visionPortal.resumeStreaming();
    }
    void stop(){
        visionPortal.stopStreaming();
    }
    void close(){
        visionPortal.close();
    }
    private double forcaFrente,forcaLado;
    private void correcaoOrientadaACampo(double drive, double turn, double theta, boolean sim){
        if (sim) {
            forcaFrente = Math.cos(theta) * drive - Math.sin(theta) * turn;
            forcaLado = Math.sin(theta) * drive + Math.cos(theta) * turn;
        }
    }
    List<ColorBlobLocatorProcessor.Blob> blobsRoxos(){
        return processorPurple.getBlobs();
    }
    ColorBlobLocatorProcessor.Blob blobRoxo(){
        if(blobsRoxos() != null){
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,150,20000, blobsRoxos());
            for(ColorBlobLocatorProcessor.Blob b : blobsRoxos()){
                return b;
            }
        }
        return null;
    }
    List<ColorBlobLocatorProcessor.Blob> blobsVerdes(){
        return processorGreen.getBlobs();
    }
    ColorBlobLocatorProcessor.Blob blobVerde(){
        if(blobsVerdes() != null){
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,50,20000, blobsVerdes());
            for(ColorBlobLocatorProcessor.Blob b : blobsVerdes()){
                return b;
            }
        }
        return null;
    }
}
