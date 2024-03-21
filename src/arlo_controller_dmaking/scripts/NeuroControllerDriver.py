import torch
import torch.nn as nn
import torch.functional as F

NUM_RAYS = 16
NUM_SONARS = 3

class NeuroControllerDriver(nn.Module):

    def __init__(self):
        super.__init__()

        input_size = NUM_RAYS * NUM_SONARS + 2
        self.fc1 = nn.Linear(input_size, 30)
        self.fc2 = nn.Linear(30, 10)
        self.fc2 = nn.Linear(10, 2)

    def forward(self, x):

        x = F.sigmoid(self.fc1(x))
        x = F.sigmoid(self.fc2(x))
        x = F.sigmoid(self.fc3(x))

        return x

