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
from predictor import VisualizationDemo
from centermask.config import get_cfg


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


def pill_segmentation():
    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    logger = setup_logger()
    # logger.info("Arguments: " + str(args))
    cfg = setup_cfg(args)

    demo = VisualizationDemo(cfg)

    if args.input:
        if os.path.isdir(args.input[0]):
            args.input = [os.path.join(args.input[0], fname) for fname in os.listdir(args.input[0])]
        elif len(args.input) == 1:
            args.input = glob.glob(os.path.expanduser(args.input[0]))
            assert args.input, "The input path(s) was not found"
        for path in tqdm.tqdm(args.input, disable=not args.output):
            # use PIL, to be consistent with evaluation
            img = read_image(path, format="BGR")
            img_sem_seg = copy.deepcopy(img)
            img_sem_seg = np.swapaxes(img_sem_seg, 0,2)
            img_sem_seg = np.swapaxes(img_sem_seg, 1,2)
            img_sem_seg = img_sem_seg[0]
            img_ins_seg = copy.deepcopy( img_sem_seg)
            start_time = time.time()
            predictions, visualized_output = demo.run_on_image(img)
            
            len_instance = len(predictions['instances'])

            #sem seg
            img_sem_seg[img_sem_seg > 0] =1
            for i in range(len_instance):
                single_mask = predictions['instances'][i].get_fields()['pred_masks'].cpu().numpy()
                single_mask = single_mask[0]
                pred_class = predictions['instances'][i].get_fields()['pred_classes'].cpu().numpy()[0]
                img_sem_seg[single_mask == True ] = (pred_class + 2) * 1
                

            #ins_seg
            img_ins_seg[img_ins_seg > 0] =1
            for i in range(2,len_instance+2):
                single_mask = predictions['instances'][i-2].get_fields()['pred_masks'].cpu().numpy()
                single_mask = single_mask[0]
                img_ins_seg[single_mask == True ] = i * 1

            
            logger.info(
                "{}: detected {} instances in {:.2f}s".format(
                    path, len(predictions["instances"]), time.time() - start_time
                )
            )

            if args.output:
                if os.path.isdir(args.output):
                    assert os.path.isdir(args.output), args.output
                    out_filename = os.path.join(args.output, os.path.basename(path))
                else:
                    assert len(args.input) == 1, "Please specify a directory with args.output"
                    out_filename = args.output
                visualized_output.save(out_filename)
                cv2.imwrite('img_ins_seg.png', img_ins_seg)
                cv2.imwrite('img_sem_seg.png', img_sem_seg)
                plt.subplot(1, 2, 1), plt.imshow(img_ins_seg)
                plt.subplot(1, 2, 2), plt.imshow(img_sem_seg)
                plt.show()
            else:
                cv2.imshow("COCO detections", visualized_output.get_image()[:, :, ::-1])
                if cv2.waitKey(0) == 27:
                    break  # esc to quit



if __name__ == "__main__":
    pill_segmentation()
    
