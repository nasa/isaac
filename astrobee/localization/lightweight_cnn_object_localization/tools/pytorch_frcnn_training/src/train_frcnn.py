


# Python imports
import argparse
from typing import Optional
import os

# Third party imports
import torch
import tqdm

# Local imports
from dataset import KeypointsDataset
from model import get_model



def main(
        n_epochs: int, 
        dataset_path: str, 
        output_path: str, 
        output_label: Optional[str] = None, 
        init_weights_path: Optional[str] = None):
    
    # =========================================================================
    # Hyperparameters
    # =========================================================================

    # training dataset
    bbox_size = 3

    # data loading
    batch_size = 2
    num_workers = 4

    # optimizer
    learning_rate = 0.0001
    momentum = 0.9
    weight_decay = 0.0005

    # learning rate scheduling
    warmup_factor = 1.0 / 1000
    warmup_iters = 1000

    # checkpointing
    checkpoint_interval = 10

    # =========================================================================
    # set up for training
    # =========================================================================

    # use gpu if available
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # 100% training for now
    
    # prepare dataset
    dataset_train = KeypointsDataset(dataset_path, is_train=True, bbox_size=bbox_size)
    # indices = torch.randperm(len(dataset_train)).tolist()
    # cutoff = int(len(dataset_train) * 0.8)
    # dataset_train = torch.utils.data.Subset(dataset_train, indices[:cutoff])
    # dataset_test = KeypointsDataset(dataset_path, is_train=False, bbox_size=bbox_size)
    # dataset_test = torch.utils.data.Subset(dataset_test, indices[cutoff:])
    
    # define training and validation data loaders
    def collate_fn(batch):
        return tuple(zip(*batch))
    data_loader_train = torch.utils.data.DataLoader(
        dataset_train,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        collate_fn=collate_fn,
    )
    # data_loader_test = torch.utils.data.DataLoader(
    #     dataset_test,
    #     batch_size=batch_size,
    #     shuffle=False,
    #     num_workers=num_workers,
    #     collate_fn=collate_fn,
    # )

    # our dataset has two classes (background, plus keypoint; all keypoints are the same right now)
    num_classes = 2

    # construct the model and move to device
    model = get_model(num_classes, weights_path=init_weights_path)
    model.to(device)

    # construct an optimizer and learning rate scheduler
    # TODO Why not try Adam? Also I have no idea why any of these hyperparameters were selected
    optimizer_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(  
        optimizer_params,
        lr=learning_rate,
        momentum=momentum,
        weight_decay=weight_decay,
    )

    # construct learning rate schedulers
    warmup_iters = min(warmup_iters, len(data_loader_train) - 1)
    def warmup_function(x):
        if x >= warmup_iters:
            return 1
        alpha = float(x) / warmup_iters
        return warmup_factor * (1 - alpha) + alpha
    lr_scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, warmup_function)

    # =========================================================================
    # training loop
    # =========================================================================

    for epoch in range(n_epochs):
        print("\n---- Training Model ----")
        model.train()
        for _, (images, targets) in enumerate(tqdm.tqdm(data_loader_train, desc=f"Training Epoch {epoch}")):

            # move data to training device
            images = list(image.to(device) for image in images)
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            # backpropagation
            loss_dict = model(images, targets)
            losses = sum(loss for loss in loss_dict.values())
            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            # update learning rate 
            if lr_scheduler is not None:
                lr_scheduler.step()

        # save checkpoints
        if ((epoch + 1) % checkpoint_interval == 0) or (epoch == (n_epochs - 1)):
            checkpoint_path = os.path.join(output_path, f"{'unlabelled' if output_label is None else output_label}_ckpt_{epoch}.pth")
            print(f"---- Saving checkpoint to: '{checkpoint_path}' ----")
            torch.save(model.state_dict(), checkpoint_path)

    print("Training complete")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--n_epochs", type=int, required=True)
    parser.add_argument("-d", "--dataset_path", type=str, required=True)
    parser.add_argument("-o", "--output_path", type=str, required=True)
    parser.add_argument("-l", "--output_label", type=str, default=None)
    parser.add_argument("-w", "--init_weights_path", type=str, default=None)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        n_epochs=args.n_epochs,
        dataset_path=args.dataset_path, 
        output_path=args.output_path,
        output_label=args.output_label,
        init_weights_path=args.init_weights_path)