class Config:

    # TODO: I'm not sure about these hyperparams, maybe do a grid search to optimize them

    # model configuration
    num_classes = 2
    # model_name = "FASTERRCNN_MOBILENET_V3_LARGE_FPN"         # 4.49 GFLOPS, 32s/iter on Astrobee
    model_name = "FASTERRCNN_MOBILENET_V3_LARGE_320_FPN"     # 0.72 GFLOPS, 14s/iter on Astrobee

    # training dataset
    bbox_size = 3

    # data loading
    batch_size = 2
    num_workers = 4

    # TODO: optimizer is currently SGD, should probably try Adam
    # optimizer
    learning_rate = 0.0001
    momentum = 0.9
    weight_decay = 0.0005

    # learning rate scheduling
    warmup_factor = 1.0 / 1000
    warmup_iters = 1000

    # checkpointing
    checkpoint_interval = 10
