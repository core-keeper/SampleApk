package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.Context;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class YOLOSharedObjectDetector extends ObjectDetector {
    private YOLODetectionService yoloService;

    public YOLOSharedObjectDetector(Context context) {
        this.yoloService = new YOLODetectionService(context);
    }

    @Override
    public List<ItemInfo> detect(Mat inputMat, String imageType) {
        List<ItemInfo> items = new ArrayList<>();

        Mat resizedMat = resizeMat(inputMat, 320, 320, Imgproc.INTER_CUBIC);
        Map<Integer, Integer> resultItems = yoloService.getItemCounts(resizedMat, imageType);
        for (Map.Entry<Integer, Integer> entry : resultItems.entrySet()) {
            items.add(new ItemInfo(yoloService.getClassName(entry.getKey()), entry.getValue()));
        }

        resizedMat.release();

        return items;
    }

    private Mat resizeMat(Mat inputMat, int targetWidth, int targetHeight, int interpolationMethod) {
        // 檢查輸入 Mat 是否為空
        if (inputMat.empty()) {
            System.err.println("錯誤：輸入 Mat 為空，無法調整大小。");
            return new Mat(); // 返回一個空的 Mat
        }

        // 創建目標尺寸對象
        Size targetSize = new Size(targetWidth, targetHeight);

        // 創建一個新的 Mat 對象來存放調整大小後的圖片
        Mat resizedMat = new Mat();

        // 執行圖片大小調整
        Imgproc.resize(inputMat, resizedMat, targetSize, 0, 0, interpolationMethod);

        return resizedMat;
    }
}
