import torch
import torch.nn as nn
import torch.nn.functional as F

import pytorch_lightning as pl
from pytorch_lightning.loggers import TensorBoardLogger

from ncps.wirings import AutoNCP, NCP
from ncps.torch import LTC


class CNNModel(nn.Module):
    def __init__(self, latent_features, dropout_rate=0.1):
        super(CNNModel, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2, padding=2)
        self.conv2 = nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2, padding=2)
        self.conv4 = nn.Conv2d(in_channels=48, out_channels=64, kernel_size=3, stride=1, padding=1)
        self.conv5 = nn.Conv2d(in_channels=64, out_channels=16, kernel_size=3, stride=2, padding=1)

        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(64 * 18 * 32, latent_features)
        self.dropout = nn.Dropout(dropout_rate)
        # self.latent = nn.Linear(2048, latent_features)

        self.predictor = nn.Sequential(
            nn.Linear(latent_features, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

        wiring = AutoNCP(19, 2)  # 16 units, 1 motor neuron
        self.ltc_model = LTC(latent_features, wiring, batch_first=True, mixed_memory=False, ode_unfolds=6)

    def forward(self, x):
        b, s, c, w, h = x.size()

        # TimeDistributed
        x = x.view(-1, c, w, h)
        x = self.conv1(x)
        x = F.relu(x)
        x = self.conv2(x)
        x = F.relu(x)
        x = self.conv3(x)
        x = F.relu(x)
        x = self.conv4(x)
        x = F.relu(x)
        x = x.view(x.size(0), -1)  # Flatten the tensor
        x = self.fc1(x)
        x = self.dropout(x)

        x, _ = self.ltc_model(x)
        x = x.view(b, s, -1)

        return x


# LightningModule for training
class CNNModule(pl.LightningModule):
    def __init__(self, hparams):
        super(CNNModule, self).__init__()
        self.model = CNNModel(latent_features=hparams['latent_features'], dropout_rate=hparams['dropout'])
        self.on_step = False
        self.on_epoch = True
        self.save_hyperparameters(hparams)
        self.__loss = nn.MSELoss()

    def forward(self, x):
        return self.model(x)

    def training_step(self, batch, batch_idx):
        images, targets = batch
        predictions = self(images)
        loss = self.__loss(predictions, targets)

        v_loss = (predictions[:, 0] - targets[:, 0]).abs().mean()
        w_loss = (predictions[:, 1] - targets[:, 1]).abs().mean()

        self.log('train_loss', loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)
        self.log('train_v_loss', v_loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)
        self.log('train_w_loss', w_loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)

        return loss

    def validation_step(self, batch, batch_idx):
        images, targets = batch
        predictions = self(images)
        loss = self.__loss(predictions, targets)

        v_loss = (predictions[:, 0] - targets[:, 0]).abs().mean()
        w_loss = (predictions[:, 1] - targets[:, 1]).abs().mean()

        self.log('test_loss', loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)
        self.log('test_v_loss', v_loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)
        self.log('test_w_loss', w_loss, on_step=self.on_step, on_epoch=self.on_epoch, prog_bar=True)

        return loss

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.hparams.learning_rate)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer=optimizer, T_max=(self.hparams.max_epochs - 1),
                                                               eta_min=1e-3 * self.hparams.learning_rate)
        return [optimizer], [scheduler]