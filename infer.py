import sys
import cv2 
import imutils
from yolov8_det_vid import YoLov8TRT

# use path for library and engine file
engine_file_path = "/home/wego/tensorrtx/yolov8/build/yolov8s.engine"
library = "/home/wego/tensorrtx/yolov8/build/libmyplugins.so"
model = YoLov8TRT(library, engine_file_path)

cap = cv2.VideoCapture(0)

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov8 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = (255, 0 ,0)#color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )
while True:
    ret, frame = cap.read()
    frame = imutils.resize(frame, width=640, height = 640)
    result_boxes, result_scores, result_classid = model.Inference(frame)
    det_res = []
    for j in range(len(result_boxes)):
        box = result_boxes[j]
        plot_one_box(box, frame, label="{}:{:.2f}".format(model.categories[int(result_classid[j])], result_scores[j]),)
        
    # for obj in detections:
    #    print(obj['class'], obj['conf'], obj['box'])0
    # print("FPS: {} sec".format(1/t))
    cv2.imshow("Output", frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
