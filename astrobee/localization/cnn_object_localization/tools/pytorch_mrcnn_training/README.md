### Running Mask-RCNN training and fine-tuning:

1. Ensure prerequisites for python are available (torchvision, tqdm, pycocotools, opencv-python)
2. `python3 train.py -n 61 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_sim"`
3. `python3 train.py -n 200 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_finetune" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_sim_ckpt_60.pth"`

### Running Mask-RCNN inference:

- `python3 test.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199.pth"`

### Converting model to TorchScript:

- `python3 convert_to_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199.pth"`

### Testing TorchScript model in Python:

- `python3 test_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199_torchscript.pt"`
