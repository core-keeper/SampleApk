package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;

import java.util.List;

public abstract class ObjectDetector {
    public abstract List<ItemInfo> detect(Mat inputMat);
}
