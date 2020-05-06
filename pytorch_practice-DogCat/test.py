

import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
from torch.autograd import Variable
import torchvision
from torchvision import datasets, models, transforms
import time
import os
from data.dataset import DogCat
from torch.utils.data import DataLoader


model=models.resnet34(pretrained=True)
num_ftrs = model.fc.in_features
model.fc = nn.Linear(num_ftrs, 2)

model.load_state_dict(torch.load('checkpoints/DogCat_model.pth'))
model.cuda()
model.eval()


transform_test = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
])

testset    = DogCat('./dataset/test1' ,transform=transform_test ,train=False, test=True)
testloader = torch.utils.data.DataLoader(testset, batch_size=96, shuffle=False, num_workers=4)


def test():

    total = 0
    correct = 0

    for batch_idx, (inputs, labels) in enumerate(testloader):
        inputs = Variable(inputs.cuda())
        labels = Variable(labels.cuda())

        outputs = model(inputs)
        _, predicted = torch.max(outputs.data, 1)

        total += inputs.size(0)
        correct += torch.sum(predicted == labels.data)

        print("TestTest  [%d|%d] " % (batch_idx, len(testloader)))

    print("Test accuracy: %f " % (1.0 * correct / total))


if __name__=='__main__':

    test()
