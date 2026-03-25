#python file for listing all pose2d and pose3d models in mmpose

from mmpose.apis import MMPoseInferencer

# Fetch the master list of all models
all_models = MMPoseInferencer.list_models('mmpose')

# 3. Filter for 3D Wholebody models (for pose3d parameter)
wholebody_coco = [m for m in all_models if 'coco' in m]


print("\n-------- COCO DATASET ------")
for model in wholebody_coco:
    print(model)
    with open("coco_models.txt", "a") as f:
        f.write(model + "\n")

# Model filename structure (OpenMMLab / MMPose convention):
# td-hm_hrnet-w48_8xb64-20e_posetrack18-384x288
#
# td-hm        -> Top-Down (td) Heatmap-based (hm) pose estimator
# hrnet-w48    -> HRNet backbone with width=48 (larger = more capacity)
# 8xb64        -> Training used 8 GPUs × batch size 64 per GPU (total batch=512)
# 20e          -> Trained for 20 epochs
# posetrack18  -> Trained on PoseTrack 2018 dataset
# 384x288      -> Input resolution (height x width)
#
# General pattern:
# {method}_{backbone}_{gpu x batch}-{epochs}_{dataset}_{input_resolution}
