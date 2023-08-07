# Mask-RCNN for handrail detection

This tool contains scripts for training and testing the Mask-RCNN, as well as converting it to TorchScript and testing the converted model in Python.
For testing the TorchScript model in a C++ environment, see the `libtorch_mrcnn_test` tool.

The training for Mask-RCNN relies on features from modern Torch/TorchVision. However, the Astrobee is constrained to use `torch=1.5.0` and therefore it is necessary to downgrade to this version of torch via venv before converting the model to TorchScript, since newer versions of torch generate non-backwards-compatible versions of TorchScript.

## Running Mask-RCNN training and fine-tuning:

1. Create and activate a venv with `torch==2.0.1 torchvision==0.15.2 tqdm pycocotools opencv-python`. Other versions of torch/torchvision may work, but this is untested.
2. `python3 train_mrcnn.py -n 61 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_mrcnn_sim"`
3. `python3 train_mrcnn.py -n 200 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_mrcnn_finetune" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_mrcnn_sim_ckpt_60.pth"`
4. `python3 test_mrcnn.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_mrcnn_finetune_ckpt_199.pth"`

## Converting model to TorchScript and testing it in Python:

1. Create and activate a venv with `torch==1.5.0 torchvision==0.6.0`.
2. `python3 convert_mrcnn_to_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_mrcnn_finetune_ckpt_199.pth"`
3. `python3 test_mrcnn_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_mrcnn_finetune_ckpt_199_torchscript.pt"`

# Keypoint-RCNN for handrail detection (EXPERIMENTAL)

This tool also contains scripts for training and testing an experimental keypoint-based version of RCNN. Note that this does not use the actual Keypoint-RCNN architecture, but instead just trains a Mask-RCNN on data containing keypoint masks. To train it on synthetic data and then test it:

1. Create and activate a venv with `torch==2.0.1 torchvision==0.15.2 tqdm pycocotools opencv-python`. Other versions of torch/torchvision may work, but this is untested.
2. `python3 train_kprcnn.py -n 61 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_kprcnn_sim"`
3. `python3 test_mrcnn.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_kprcnn_sim_ckpt_60.pth"`
