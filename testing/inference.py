import os
import time

import torch

from PIL import Image

import numpy as np
import csv

import cv2

from natsort import os_sorted

from net import CNNModule

def main():
    hparams = {
        "learning_rate": 5e-3,
        "batch_size": 1,
        "num_workers": 8,
        "num_classes": 2,
        'max_epochs': 300,
        'dropout': 0.1,
        'latent_features': 128,
        'da': True,
    }

    model: CNNModule = CNNModule(hparams)
    # model_st_ = torch.load("../models/epoch=12-step=13.ckpt", map_location=torch.device("cpu"))
    # model.load_state_dict(model_st_["state_dict"])
    model = model.to("cpu")
    model.train()

    seq: torch.Tensor = torch.zeros(1, 1, 3, 144, 256)


    images_paths: str = os_sorted(os.listdir("../data/video11"))[:-1]

    fps_list: list[float] = []

    for path in images_paths:
        image: Image.Image = Image.open(os.path.join("../data/video11", path))

        cv2.imshow("", np.array(image))
        cv2.waitKey(1)

        image = np.resize(image, (144, 256, 3))

        t0: float = time.time()
        
        numpy_image = np.float32(image) / 255
        tensor = torch.tensor(numpy_image).permute(2, 0, 1)
        
        seq[0, 0:-1, ...] = seq[0, 1:, ...].clone()
        seq[0, -1, ...] = tensor
        
        y_hat = model(seq)

        t1: float = time.time()

        fps: float = round(1 / (t1 - t0), 3)
        fps_list.append(fps)

        print(y_hat)
    
    print(f"Average FPS: {np.mean(fps_list)}")


if __name__ == '__main__':
    main()
