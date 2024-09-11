
import pyrealsense2 as rs
import numpy as np
import cv2
# YOLO model
from ultralytics import YOLO
# pointcloud reconstruction
import open3d as o3d
from sphere_ransac import Sphere

model = YOLO("train6/weights/best.pt")  # pretrained YOLOv8n model
def segment_apples(image, confidence_thresh=.6, debug=False):
    # returns masks of apples, each apple has its own mask. 0 for no apple, 255 for apple.  
    # predict segmentation masks using model
    results = model(image, conf=confidence_thresh)[0]
    apple_masks = []

    # creates a mask for each apple detected over the original image
    for i in results:
        # create empty mask
        img_h, img_w = results.masks.orig_shape
        mask = np.zeros((img_h, img_w), dtype=np.uint8)
        # fill in mask with white where predicted apple segmentation is
        cv2.fillPoly(mask, np.int32([i.masks.xy]), (255, 255, 255))
        apple_masks.append(mask)
    # results.show()  # display to screen
    # Optional visualization for debugging
    if debug: 
        results.show()  # display to screen
        for i in apple_masks:
            cv2.imshow("apple_masks", i)
            key = cv2.waitKey(0)
            print("Press esc to view next apple mask.")
            if key == 27:
                cv2.destroyAllWindows()
    return apple_masks

# Create a pipeline
pipeline = rs.pipeline()
# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # images = np.hstack((color_image, depth_image))
        # segment_apples(color_image)
        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', color_image)
        cv2.imshow('Align Example2', depth_image)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
