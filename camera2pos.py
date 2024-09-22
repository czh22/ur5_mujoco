from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import threading




class cam2pos:


    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # 打开默认摄像头，可以根据需求选择其他摄像头索引
        if not self.cap.isOpened():
            raise ValueError("无法打开摄像头")

        self.base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
        self.options = vision.PoseLandmarkerOptions(
            base_options=self.base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(self.options)
        
        self.pos = [0, 0, 0]

        # 创建并启动线程
        self.thread = threading.Thread(target=self._camera_thread)
        self.thread.start()

    def draw_landmarks_on_image(self, rgb_image, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        # Loop through the detected poses to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                pose_landmarks_proto,
                solutions.pose.POSE_CONNECTIONS,
                solutions.drawing_styles.get_default_pose_landmarks_style())
        return annotated_image


    def infer_pos(self):
        ret, img = self.cap.read()
        if cv2.waitKey(1) == ord('q'):  # 按下 'q' 键退出循环
            self.cap.release()  # 关闭摄像头
            cv2.destroyAllWindows()  # 关闭窗口
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
        detection_result = self.detector.detect(image)
        annotated_image = self.draw_landmarks_on_image(image.numpy_view(), detection_result)
        cv2.imshow('res', annotated_image)

        if len(detection_result.pose_world_landmarks) > 0:
            # print(detection_result.pose_world_landmarks[0][0])
            return [detection_result.pose_world_landmarks[0][16].x - detection_result.pose_world_landmarks[0][12].x,
                    (detection_result.pose_world_landmarks[0][16].z - detection_result.pose_world_landmarks[0][12].z), 
                    -(detection_result.pose_world_landmarks[0][16].y - detection_result.pose_world_landmarks[0][12].y)]

        else:
            return None
        

    def _camera_thread(self):
        while True:
            pos = self.infer_pos()
            if pos is not None:
                self.pos = pos
          

    def get_pos(self):
        return self.pos.copy()




# 在文件末尾添加以下测试函数
def test_cam2pos():
    import time
    
    # 初始化 cam2pos 对象
    camera = cam2pos()
    
    print("摄像头初始化完成,开始测试...")
    
    # 测试持续 10 秒
    start_time = time.time()
    while time.time() - start_time < 10:
        pos = camera.get_pos()
        if pos is not None:
            print(f"检测到姿势: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
        else:
            print("未检测到姿势")
        time.sleep(0.2)  # 每0.5秒检查一次
    
    print("测试完成")

if __name__ == "__main__":
    test_cam2pos()



