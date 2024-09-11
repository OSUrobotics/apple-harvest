# vision
import numpy as np
import cv2
# YOLO model
from ultralytics import YOLO
# pointcloud reconstruction
import open3d as o3d
from sphere_ransac import Sphere
import pyrealsense2 as rs

model = YOLO("train6/weights/best.pt")  # pretrained YOLOv8n model
image_path = "harvest_test/rgb/rgb_frame_0.png"
# realsense setup
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



def take_picture():
    # Start streaming
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    flush_count = 0
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        # aligned_depth_frame = rs.spatial_filter().process(aligned_depth_frame)
        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
        elif flush_count < 30:
            flush_count += 1
            continue
        else:
            print("HERE")
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            segment_apples(color_image)
            break
    pipeline.stop()
    # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    # cv2.imshow('Align Example', color_image)
    # cv2.imshow('Align Example2', depth_image)
    # key = cv2.waitKey(0)
    # # Press esc or 'q' to close the image window
    # if key & 0xFF == ord('q') or key == 27:
    #     cv2.destroyAllWindows()
    return color_image, depth_image


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



def ransac_apple_estimation(pcd):
    center = None
    radius = None
    sph_ransac = Sphere()
    center, radius, inliers = sph_ransac.fit(pcd, thresh=.0001, maxIteration=10000, lower_rad_bound=.03, upper_rad_bound=.06)

    print(radius, center)
    return center, radius


def get_apple_centers(rgb, depth, masks):
    #TODO check that apple is far enough, check that apples have been identified. 
    apple_centers = []
    apple_radii = []
    visualization = []
    for mask in masks: 
        # segment only the apple portions
        depth_segmented = np.where(mask, depth, 0)
        # create RGBD image (NEEDS TO BE IN RGB FORMAT NOT BGR TO LOOK RIGHT, doesnt super matter for anything other than visualization)
        rgb_pc = o3d.geometry.Image(rgb)
        depth_pc = o3d.geometry.Image(depth_segmented)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_pc, depth_pc, convert_rgb_to_intensity=False)
        # Creates pointcloud using camera intrinsics from Realsense 435i
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(width=848, height=480, fx=609.6989, fy=609.8549, cx=420.2079, cy=235.2782))
        # Transforms pointcloud to be facing the correct direction.
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # o3d.visualization.draw_geometries([pcd])
        center, radius = ransac_apple_estimation(np.array(pcd.points))

        if center and radius: 
            mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            mesh_sphere.compute_vertex_normals()
            mesh_sphere.paint_uniform_color([1,0,0])
            mesh_sphere.translate(np.array(center), relative=False)
            apple_centers.append(center)
            apple_radii.append(radius)
            visualization.append(pcd)
            visualization.append(mesh_sphere)
            
    
        # visualization.append(pcd)
    # o3d.visualization.draw_geometries(visualization)

    rgb_pc = o3d.geometry.Image(rgb)
    depth_pc = o3d.geometry.Image(np.array(depth, dtype=np.float32))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_pc, depth_pc, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(width=848, height=480, fx=609.6989, fy=609.8549, cx=420.2079, cy=235.2782))
    # Transforms pointcloud to be facing the correct direction.
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    o3d.visualization.draw_geometries([pcd] + visualization)
    return apple_centers, apple_radii, visualization

rgb_image, depth_image = take_picture()
apple_masks = segment_apples(rgb_image, confidence_thresh=.7, debug=True)
rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
centers, radii, vis = get_apple_centers(rgb_image, depth_image, apple_masks)
# o3d.visualization.draw_geometries(vis)
# img = cv2.imread(image_path)

# depth_image = cv2.imread('harvest_test/depth/depth_frame_0.tiff', cv2.IMREAD_UNCHANGED) 
# print(depth_image)
# apple_masks = segment_apples(img, confidence_thresh=.6, debug=False)
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# get_apple_centers(img, depth_image, apple_masks)

