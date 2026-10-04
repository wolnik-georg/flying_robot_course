#!/usr/bin/env python3
"""NeuralSwarm2 force model (ROS-free excerpt from CS2 neuralswarm.py)."""

from __future__ import annotations

from pathlib import Path

import warnings

import numpy as np
import torch

warnings.filterwarnings("ignore", category=FutureWarning, module="indi_sil_package_ns2")
import torch.nn as nn
import torch.nn.functional as F


class phi_Net(nn.Module):
    def __init__(self, inputdim=6, hiddendim=40):
        super().__init__()
        self.fc1 = nn.Linear(inputdim, 25)
        self.fc2 = nn.Linear(25, 40)
        self.fc3 = nn.Linear(40, 40)
        self.fc4 = nn.Linear(40, hiddendim)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        return self.fc4(x)


class rho_Net(nn.Module):
    def __init__(self, hiddendim=40):
        super().__init__()
        self.fc1 = nn.Linear(hiddendim, 40)
        self.fc2 = nn.Linear(40, 40)
        self.fc3 = nn.Linear(40, 40)
        self.fc4 = nn.Linear(40, 1)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        return self.fc4(x)


class NeuralSwarm:
    def __init__(self, model_folder: Path):
        self.H = 20
        mf = str(model_folder)
        self.rho_L_net = rho_Net(hiddendim=self.H)
        self.phi_L_net = phi_Net(inputdim=6, hiddendim=self.H)
        self.rho_L_net.load_state_dict(torch.load(f"{mf}/rho_L.pth"))
        self.phi_L_net.load_state_dict(torch.load(f"{mf}/phi_L.pth"))
        self.rho_S_net = rho_Net(hiddendim=self.H)
        self.phi_S_net = phi_Net(inputdim=6, hiddendim=self.H)
        self.rho_S_net.load_state_dict(torch.load(f"{mf}/rho_S.pth"))
        self.phi_S_net.load_state_dict(torch.load(f"{mf}/phi_S.pth"))
        self.phi_G_net = phi_Net(inputdim=4, hiddendim=self.H)
        self.phi_G_net.load_state_dict(torch.load(f"{mf}/phi_G.pth"))

    def compute_Fa(self, data_self, data_neighbors):
        rho_input = torch.zeros(self.H)
        cftype, x = data_self
        for cftype_neighbor, x_neighbor in data_neighbors:
            x_12 = (x_neighbor - x).float()
            if abs(x_12[0]) < 0.2 and abs(x_12[1]) < 0.2 and abs(x_12[3]) < 1.5:
                if cftype_neighbor in ("small", "small_powerful_motors"):
                    rho_input += self.phi_S_net(x_12)
                elif cftype_neighbor == "large":
                    rho_input += self.phi_L_net(x_12)
                else:
                    raise Exception("Unknown cftype!")

        x_12 = torch.zeros(4)
        x_12[0] = 0 - x[2]
        x_12[1:4] = -x[3:6]
        rho_input += self.phi_G_net(x_12)

        if cftype in ("small", "small_powerful_motors"):
            faz = self.rho_S_net(rho_input)
        elif cftype == "large":
            faz = self.rho_L_net(rho_input)
        else:
            raise Exception("Unknown cftype!")
        return np.array([0, 0, faz[0].item()])
