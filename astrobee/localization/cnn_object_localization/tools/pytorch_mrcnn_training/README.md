### Running Mask-RCNN training and fine-tuning:

1. Ensure prerequisites for python are available (torchvision, tqdm, pycocotools, opencv-python)
2. `python3 train.py -n 61 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_sim"`
3. `python3 train.py -n 200 -d "PATH/TO/SIMULATED/DATA" -o "PATH/TO/CHECKPOINTS/FOLDER" -l "handrail_finetune" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_sim_ckpt_60.pth"`

### Running Mask-RCNN inference:

- `python3 test.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199.pth"`

### Converting model to TorchScript:

- Create and activate a venv with `torch==1.5.0 torchvision==0.6.0` (necessary to save version 3 TorchScript models, as ISAAC's version of libtorch isn't compatible with anything newer)
- `python3 convert_to_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199.pth"`

### Testing TorchScript model in Python:

- Activate the previously created venv
- `python3 test_torchscript.py -d "PATH/TO/IMAGES/FOLDER" -o "PATH/TO/OUTPUTS/FOLDER" -w "PATH/TO/CHECKPOINTS/FOLDER/handrail_finetune_ckpt_199_torchscript.pt"`
