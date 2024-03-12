# CNN for handrail detection

This tool contains scripts for training and testing the keypoint detector CNN, as well as converting it to TorchScript and testing the converted model in Python.
For testing the TorchScript model in a C++ environment, see the `libtorch_cnn_test` tool.

Two different model architectures can be trained: 

- `FASTERRCNN_MOBILENET_V3_LARGE_FPN` runs at a rate of 32s/iter on Astrobee.
- `FASTERRCNN_MOBILENET_V3_LARGE_320_FPN` runs at a rate of 14s/iter on Astrobee.

Training, testing, conversion to TorchScript, and inference in libtorch all use `torch==1.13.1 torchvision==0.14.1`. 
This is the final 1.X version of Torch.
For a full list of Python dependencies for training and testing, see `requirements.txt`.

NOTE: In its current state, this model training code is a quick-and-dirty experiment. 
As such, keep the following in mind:
- Most of the hyperparameters and training methodology were arbitrarily copied over from what was used for Mask-RCNN with a ResNet-50 backbone (the older, too-large-for-Astrobee model). This tool trains very different models, and as such there's really no reason to assume any of the hyperparameters or training methodologies are still optimal.
- The training code supports fine-tuning on manually labelled real-world data; however, I have not manually labelled any such data yet. So you're stuck using the model trained on simulated data for now.
- There is currently no implementation of validation, testing, metrics, etc. All evaluation is based on empirical results. This is definitely a huge to-do item.
- The model configuration and training hyperparameters used can be modified in `src/config.py`. If you're doing experiments in an attempt to optimize the training configuration, this is pretty awful and you might want to implement a cleaner way of specifying configurations, such as through config files.

## Running CNN training, fine-tuning, and inference:

- Training on simulated data: `python3 src/train_cnn.py -n 60 -d <PATH/TO/SIMULATED/DATASET> -o <PATH/TO/CHECKPOINTS/FOLDER> -l <LABEL_FOR_OUTPUT_CHECKPOINTS>`
- (Hypothetical) fine tuning: `python3 src/train_cnn.py -n 200 -d <PATH/TO/FINETUNING/DATASET> -o <PATH/TO/CHECKPOINTS/FOLDER> -l <LABEL_FOR_OUTPUT_CHECKPOINTS> -w <PATH/TO/INITIAL/WEIGHTS>`
- Standalone inference: `python3 src/test_cnn.py -d <PATH/TO/INPUT/IMAGES> -o <PATH/TO/OUTPUTS/FOLDER> -w <PATH/TO/CHECKPOINT/FILE>`

## Converting model to TorchScript and testing it in Python:

- `python3 convert_cnn.py -d <PATH/TO/INPUT/IMAGES> -w <PATH/TO/CHECKPOINT/FILE>`
