package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.Context;
import android.util.Log;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfRect2d;
import org.opencv.core.Rect2d; // 即使不繪圖，NMS 仍需要它
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class YOLOObjectDetector extends ObjectDetector {

    private static final String TAG = "ObjectDetector";
    private static final String MODEL_FILE = "yolov8n_finetune.onnx"; // 放在 assets 資料夾中的模型檔名
    private static final int INPUT_SIZE = 224; // <--- 模型輸入尺寸，必須是 224
    private static final float CONF_THRESHOLD = 0.25f; // 物體置信度閾值
    private static final float NMS_THRESHOLD = 0.45f;  // NMS (非最大值抑制) 閾值

    // 定義 YOLOv8 的類別名稱。順序必須與你模型訓練時的類別 ID 順序一致。
    private static final List<String> CLASS_NAMES = Arrays.asList(
            "coin", "compass", "coral", "crystal",
            "diamond", "emerald", "fossil", "key",
            "letter", "shell", "treasure_box"
    );

    private Net net; // OpenCV DNN 網路物件

    /**
     * 建構子：初始化 ObjectDetector 並載入模型。
     * 請確保在呼叫此建構子之前，OpenCVLoader.initDebug() 已成功初始化。
     *
     * @param context 應用程式的 Context，用於存取 assets。
     */
    public YOLOObjectDetector(Context context) {
        if (!OpenCVLoader.initDebug()) {
            Log.e(TAG, "OpenCV 初始化失敗！請確保你已在 Activity/Application 中正確呼叫了 OpenCVLoader.initDebug()。");
            // 你可以選擇在這裡拋出 RuntimeException 或通過回呼通知調用者。
        } else {
            Log.d(TAG, "OpenCV 模組已就緒。");
            loadModel(context); // 傳入 context 以便訪問 assets
        }
    }

    private void loadModel(Context context) {
        try {
            // 從 assets 複製模型檔到內部儲存空間 (OpenCV Dnn.readNetFromONNX 需要檔案路徑)
            String modelPath = context.getCacheDir().getAbsolutePath() + File.separator + MODEL_FILE;
            File modelFile = new File(modelPath);

            if (!modelFile.exists()) { // 如果檔案不存在，則複製
                Log.d(TAG, "模型檔案不存在，正在從 assets 複製到: " + modelPath);
                InputStream is = context.getAssets().open(MODEL_FILE); // 直接使用傳入的 context 取得 assets
                FileOutputStream fos = new FileOutputStream(modelFile);
                byte[] buffer = new byte[4096]; // 4KB 緩衝區
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    fos.write(buffer, 0, bytesRead);
                }
                fos.close();
                is.close();
            } else {
                Log.d(TAG, "模型檔案已存在於內部儲存空間: " + modelPath);
            }

            net = Dnn.readNetFromONNX(modelPath);
            if (net.empty()) {
                Log.e(TAG, "無法載入 ONNX 模型: " + MODEL_FILE + "。請檢查檔案是否損壞或路徑是否正確。");
                // 處理模型載入失敗情況
            } else {
                Log.d(TAG, "模型載入成功！");
            }

        } catch (IOException e) {
            Log.e(TAG, "讀取或複製模型檔時發生 IO 錯誤: " + e.getMessage(), e);
        } catch (Exception e) {
            Log.e(TAG, "載入模型時發生未知錯誤: " + e.getMessage(), e);
        }
    }

    /**
     * 對輸入的 Mat 影像執行物件檢測，並返回檢測到的物件類別及其數量。
     *
     * @param inputMat 已經是 224x224 像素的 Mat 影像。
     * @return 一個 Map，鍵為物件類別名稱 (String)，值為該類別的物件數量 (Integer)。
     * 如果檢測失敗或沒有檢測到物件，返回空 Map。
     */
    @Override
    public List<ItemInfo> detect(Mat inputMat) {
        Map<String, Integer> detectedObjectCounts = new HashMap<>();

        if (net == null || net.empty()) {
            Log.e(TAG, "模型尚未載入或載入失敗，無法執行檢測。");
            return new ArrayList<ItemInfo>(); // 返回空 List
        }

        // 檢查輸入 Mat 的尺寸是否符合模型預期
        if (inputMat.cols() != INPUT_SIZE || inputMat.rows() != INPUT_SIZE) {
            Log.e(TAG, "輸入 Mat 尺寸不符。期望 " + INPUT_SIZE + "x" + INPUT_SIZE + ", 實際 " + inputMat.cols() + "x" + inputMat.rows());
            return new ArrayList<ItemInfo>(); // 返回空 List
        }

        // 影像預處理：正規化、創建 Blob。YOLOv8 通常期望 RGB 影像 (swapRB=true)。
        Mat blob = Dnn.blobFromImage(inputMat, 1 / 255.0, new Size(INPUT_SIZE, INPUT_SIZE),
                new Scalar(0, 0, 0), true, false);

        net.setInput(blob);

        // 執行推斷
        Mat output = net.forward();
        Log.d(TAG, "模型輸出形狀: [rows=" + output.rows() + ", cols=" + output.cols() + "]");

        // --- 後處理：解析輸出、篩選、NMS ---
        // YOLOv8 的 ONNX 輸出通常是 (1, 84, N)，我們需要將其 reshape 為 (N, 84) 便於迭代。
        // 84 = (4 邊界框座標) + (1 物體置信度) + (79 類別置信度，COCO 類別數量)
        // 注意：這裡的 '84' 可能是您模型輸出特徵數量的近似值，實際值取決於您的模型。
        // 如果您的模型輸出是 (1, N, X) 形式，則 reshape 可能是 output.reshape(1, (int) output.size().width);
        // 通常對於 YOLOv8，它是 (1, num_features, num_detections)，所以 reshape(1, num_detections) 是常見的。
        Mat detectionMat = output.reshape(1, (int) output.size().height);

        List<Rect2d> boxes = new ArrayList<>(); // NMS 需要邊界框資訊
        List<Float> confidences = new ArrayList<>(); // NMS 需要置信度
        List<Integer> classIds = new ArrayList<>(); // 用於最終計數

        for (int i = 0; i < detectionMat.rows(); ++i) {
            // 每行的第 5 個元素是物件置信度
            float object_confidence = (float) detectionMat.get(i, 4)[0];

            if (object_confidence < CONF_THRESHOLD) {
                continue; // 忽略低於物體置信度閾值的檢測
            }

            // 從第 5 個元素開始是類別置信度，找到最高置信度的類別
            // 注意：這裡的 5 是一個索引，對應模型輸出的 class scores 開始位置。
            // 對於 YOLOv8 預設的 84 維輸出，5 是正確的。
            Mat scores = detectionMat.row(i).colRange(5, detectionMat.cols());
            Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
            float class_confidence = (float) mm.maxVal;
            int class_id = (int) mm.maxLoc.x; // maxLoc.x 是最大值所在的列索引

            // 計算最終的置信度 (物件置信度 * 類別置信度)
            float final_confidence = object_confidence * class_confidence;

            if (final_confidence > CONF_THRESHOLD) {
                // 即使不繪製，NMS 仍需要這些邊界框座標來判斷重疊
                float x_center = (float) detectionMat.get(i, 0)[0];
                float y_center = (float) detectionMat.get(i, 1)[0];
                float width = (float) detectionMat.get(i, 2)[0];
                float height = (float) detectionMat.get(i, 3)[0];

                // 將中心點座標和寬高轉換為左上角座標和寬高
                float x = x_center - width / 2;
                float y = y_center - height / 2;

                boxes.add(new Rect2d(x, y, width, height));
                confidences.add(final_confidence);
                classIds.add(class_id);
            }
        }

        // --- 應用 NMS (非最大值抑制) ---
        // NMS 用於過濾掉高度重疊的邊界框，確保每個物件只被計數一次。
        MatOfFloat confidencesMat = new MatOfFloat();
        confidencesMat.fromList(confidences);

        MatOfRect2d boxesMat = new MatOfRect2d();
        boxesMat.fromList(boxes);

        org.opencv.core.MatOfInt indices = new org.opencv.core.MatOfInt();
        Dnn.NMSBoxes(boxesMat, confidencesMat, CONF_THRESHOLD, NMS_THRESHOLD, indices);

        // --- 計數最終檢測到的物件類別和數量 ---
        int[] finalIndices = indices.toArray();
        Log.d(TAG, "NMS 後的總檢測數量：" + finalIndices.length);

        for (int i = 0; i < finalIndices.length; ++i) {
            int idx = finalIndices[i];
            int classId = classIds.get(idx);

            // 確保類別 ID 有效，避免索引超出範圍
            if (classId >= 0 && classId < CLASS_NAMES.size()) {
                String className = CLASS_NAMES.get(classId);
                detectedObjectCounts.put(className, detectedObjectCounts.getOrDefault(className, 0) + 1);
            } else {
                Log.w(TAG, "檢測到一個無效的類別 ID (可能超出 CLASS_NAMES 範圍)：" + classId);
            }
        }

        // 釋放中間 Mat 的記憶體，防止記憶體洩漏
        blob.release();
        output.release();
        detectionMat.release();
        confidencesMat.release();
        boxesMat.release();
        indices.release();

        List<ItemInfo> items = new ArrayList<>();
        for (Map.Entry<String, Integer> entry : detectedObjectCounts.entrySet()) {
            ItemInfo item = new ItemInfo(entry.getKey(), entry.getValue());
            items.add(item);
        }

        return items;
    }
}