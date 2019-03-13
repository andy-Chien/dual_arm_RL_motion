#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import argparse
from yolo import YOLO, detect_video
from PIL import Image
import os
from timeit import default_timer as timer
import numpy as np
from yolov3_sandwich.msg import ROI
from yolov3_sandwich.msg import ROI_array

# sys.path.insert(1,'/home/iclab/.local/lib/python3.5/site-packages')


def detect_img(yolo):
    while True:
        img = input('Input image filename:')
        try:
            image = Image.open(img)
        except:
            print('Open Error! Try again!')
            continue
        else:
            r_image = yolo.detect_image(image)
            r_image.show()
    yolo.close_session()


def clone_detect_video(yolo, video_path, output_path=""):
    print("yolo type = "+str(type(yolo)))
    import cv2
    # video_path = "/home/iclab/Downloads/Humans_HD_Stock_Video.mp4"
    video_path = 2
    vid = cv2.VideoCapture(video_path)

    if not vid.isOpened():
        raise IOError("Couldn't open webcam or video")
    video_FourCC    = int(vid.get(cv2.CAP_PROP_FOURCC))
    video_fps       = vid.get(cv2.CAP_PROP_FPS)
    video_size      = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    isOutput = True if output_path != "" else False
    if isOutput:
        print("!!! TYPE:", type(output_path), type(video_FourCC), type(video_fps), type(video_size))
        out = cv2.VideoWriter(output_path, video_FourCC, video_fps, video_size)
    accum_time = 0
    curr_fps = 0
    fps = "FPS: ??"
    prev_time = timer()
    while not rospy.is_shutdown():
        return_value, frame = vid.read()
        image = Image.fromarray(frame)
        image,ROI_recive = yolo.detect_image(image)
        if ROI_recive != None:
            ROI_msg = ROI()
            print("class_name type = " + str(ROI_recive[0]))
            print("score type = "      + str(ROI_recive[1]))
            print("x_min type = "      + str(ROI_recive[2]))
            print("x_Max type = "      + str(ROI_recive[3]))
            print("y_min type = "      + str(ROI_recive[4]))
            print("y_Max type = "      + str(ROI_recive[5]))

            ROI_msg.class_name = str(ROI_recive[0])
            ROI_msg.score      = float(ROI_recive[1])
            ROI_msg.x_min      = int(ROI_recive[2])
            ROI_msg.x_Max      = int(ROI_recive[3])
            ROI_msg.y_min      = int(ROI_recive[4])
            ROI_msg.y_Max      = int(ROI_recive[5])
            roi_pub.publish(ROI_msg)
        else:
            print("no object now")
        result = np.asarray(image)
        curr_time = timer()
        exec_time = curr_time - prev_time
        prev_time = curr_time
        accum_time = accum_time + exec_time
        curr_fps = curr_fps + 1
        if accum_time > 1:
            accum_time = accum_time - 1
            fps = "FPS: " + str(curr_fps)
            curr_fps = 0
        cv2.putText(result, text=fps, org=(3, 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.50, color=(255, 0, 0), thickness=2)
        cv2.namedWindow("result", cv2.WINDOW_NORMAL)
        cv2.imshow("result", result)
        if isOutput:
            out.write(result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

FLAGS = None
rospy.init_node('image_converter', anonymous=True)
roi_pub = rospy.Publisher("/object/ROI",ROI,queue_size=10)

if __name__ == '__main__':

    # class YOLO defines the default value, so suppress any default here
    parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
    '''
    Command line options
    '''
    parser.add_argument(
        '--model', type=str,
        help='path to model weight file, default ' + YOLO.get_defaults("model_path")
    )
    
    parser.add_argument(
        '--anchors', type=str,
        help='path to anchor definitions, default ' + YOLO.get_defaults("anchors_path")
    )

    parser.add_argument(
        '--classes', type=str,
        help='path to class definitions, default ' + YOLO.get_defaults("classes_path")
    )

    parser.add_argument(
        '--gpu_num', type=int,
        help='Number of GPU to use, default ' + str(YOLO.get_defaults("gpu_num"))
    )

    parser.add_argument(
        '--image', default=False, action="store_true",
        help='Image detection mode, will ignore all positional arguments'
    )
    '''
    Command line positional arguments -- for video detection mode
    '''
    parser.add_argument(
        "--input", nargs='?', type=str,required=False,default='./path2your_video',
        help = "Video input path"
    )

    parser.add_argument(
        "--output", nargs='?', type=str, default="",
        help = "[Optional] Video output path"
    )

    FLAGS = parser.parse_args()

    if FLAGS.image:
        """
        Image detection mode, disregard any remaining command line arguments
        """
        print("Image detection mode")
        if "input" in FLAGS:
            print(" Ignoring remaining command line arguments: " + FLAGS.input + "," + FLAGS.output)
        detect_img(YOLO(**vars(FLAGS)))
    elif "input" in FLAGS:
        # detect_video(YOLO(**vars(FLAGS)), FLAGS.input, FLAGS.output)
        clone_detect_video(YOLO(**vars(FLAGS)), FLAGS.input, FLAGS.output)
    else:
        print("Must specify at least video_input_path.  See usage with --help.")
