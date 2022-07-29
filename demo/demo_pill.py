import argparse
import glob
import multiprocessing as mp
import os
import time
import cv2
import tqdm
import sys
import numpy as np
import copy
import matplotlib.pyplot as plt

#TODO : this is a temporary expedient
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger
from demo.predictor import VisualizationDemo
from centermask.config import get_cfg
from centermask.grasp import grasp


def setup_cfg(args):
    # load config from file and command-line arguments
    cfg = get_cfg()
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    # Set score_threshold for builtin models
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.FCOS.INFERENCE_TH_TEST = args.confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
    cfg.freeze()
    return cfg


def get_parser():
    parser = argparse.ArgumentParser(description="Detectron2 Demo")
    parser.add_argument("--config-file", default="configs/centermask/centermask_V_99_eSE_dcn_FPN_ms_3x.yaml",
        metavar="FILE", help="path to config file",)
    parser.add_argument("--webcam", action="store_true", help="Take inputs from webcam.")
    parser.add_argument("--video-input", help="Path to video file.")
    parser.add_argument("--input", nargs="+", default=['input_img/'], help="A list of space separated input images")
    parser.add_argument("--output", default='results/',
        help="A file or directory to save output visualizations. "
        "If not given, will show output in an OpenCV window.",)
    parser.add_argument("--confidence-threshold", type=float, default=0.4,
        help="Minimum score for instance predictions to be shown",)
    parser.add_argument("--opts", default=['MODEL.WEIGHTS', 'model_final.pth'],
        help="Modify config options using the command-line 'KEY VALUE' pairs",
        nargs=argparse.REMAINDER,)
    return parser


def pill_segmentation(img):
    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    logger = setup_logger()
    # logger.info("Arguments: " + str(args))
    cfg = setup_cfg(args)

    # use PIL, to be consistent with evaluation
    #path = '/home/hanwen/Projects/qingwei/centermask2/input_img/06150_Color.png'
    #img = read_image(path, format="BGR")
    img_sem_seg = copy.deepcopy(img)
    img_sem_seg = np.swapaxes(img_sem_seg, 0,2)
    img_sem_seg = np.swapaxes(img_sem_seg, 1,2)
    img_sem_seg = img_sem_seg[0]
    img_ins_seg = copy.deepcopy(img_sem_seg)
    
    # Detection
    demo = VisualizationDemo(cfg)
    start_time = time.time()
    predictions, visualized_output = demo.run_on_image(img)
    len_instance = len(predictions['instances'])

    #sem seg
    img_sem_seg[img_sem_seg >= 0] =1
    for i in range(len_instance):
        single_mask = predictions['instances'][i].get_fields()['pred_masks'].cpu().numpy()
        single_mask = single_mask[0]
        pred_class = predictions['instances'][i].get_fields()['pred_classes'].cpu().numpy()[0]
        img_sem_seg[single_mask == True ] = (pred_class + 2) * 1
        
    #ins_seg
    img_ins_seg[img_ins_seg >= 0] =1
    for i in range(2,len_instance+2):
        single_mask = predictions['instances'][i-2].get_fields()['pred_masks'].cpu().numpy()
        single_mask = single_mask[0]
        img_ins_seg[single_mask == True ] = i * 1
    
    # logger.info("{}: detected {} instances in {:.2f}s".format(path, len(predictions["instances"]), time.time() - start_time))

    #out_filename = os.path.join(args.output, os.path.basename(path))
    #visualized_output.save(out_filename)

    # wipe all detections outside the manipulation area
    img_ins_seg[:,0:250] = 1
    img_ins_seg[:,1100:-1] = 1
    img_sem_seg[:,0:250] = 1
    img_sem_seg[:,1100:-1] = 1

    # do not detect gripper as pill
    img_ins_seg[0:45, 780:920] = 1
    img_sem_seg[0:45, 780:920] = 1

    return img_ins_seg, img_sem_seg



if __name__ == "__main__":
    path = '/home/hanwen/Projects/pill-sorting/qingwei/centermask2/input_img/2.jpg'
    img = read_image(path, format="BGR")
    
    img_ins_seg, img_sem_seg = pill_segmentation(img)
    
    #plt.subplot(1, 2, 1), plt.imshow(img_ins_seg)
    #plt.subplot(1, 2, 2), plt.imshow(img_sem_seg)
    #plt.show()
    
    print('\n')
    # img_gray = cv2.imread(path, 0)
    # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg, img_sem_seg)

    print('\n')
    # 0:pushing, 1:swiping, 2:picking
    if motion_command == 0:
        print('push_start:', push_start)
        print('push_end:', push_end)
    elif motion_command == 1:
        print('swipe_start:', push_start)
        print('swipe_end:', push_end)
    elif motion_command == 2:
        print('grasp_coord:', grasp_coord)
        print('grasp_angle:', grasp_angle)
        print('grasp_opening:', grasp_opening)
    else:
        print('Unknown motion type.')

    
    
    
    
