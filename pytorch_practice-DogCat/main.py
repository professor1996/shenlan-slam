# coding:utf8


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


# image preprocess
transform_train = transforms.Compose([
    transforms.Resize((256, 256)),
    transforms.RandomCrop((224, 224)),
    transforms.RandomHorizontalFlip(),
    transforms.ToTensor(),
    transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
])

transform_val = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
])

#get dataset
trainset = DogCat('./dataset/train', transform=transform_train, train=True, test=False)
valset = DogCat('./dataset/train', transform=transform_val, train=False, test=False)

#load dataset
trainloader = torch.utils.data.DataLoader(trainset, batch_size=96, shuffle=True, num_workers=4)
valloader = torch.utils.data.DataLoader(valset, batch_size=96, shuffle=True, num_workers=4)

#use gpu
use_gpu = torch.cuda.is_available()

#load model
model = models.resnet34(pretrained=True)
num_ftrs = model.fc.in_features
model.fc = nn.Linear(num_ftrs, 2)
model.cuda()

#define loss function
criterion = nn.CrossEntropyLoss()
criterion.cuda()

#define optimize method
optimizer = torch.optim.SGD(model.parameters(), lr=0.0001, momentum=0.9)  # , weight_decay=5e-4

#define learning rate strategy
scheduler = lr_scheduler.StepLR(optimizer, step_size=7, gamma=0.1)

#epoch numbers
num_epochs=1


def train(epoch):

    #update learning rate:
    scheduler.step()

    #set model as training
    model.train(True)

    #for circle for each batch of training dataset

    #return data: id , (imgs , labels)  defined in DogCat class
    for batch_idx,(inputs,labels) in enumerate(trainloader):

        #transform their data type to Variable
        inputs = Variable(inputs.cuda())
        labels = Variable(labels.cuda())

        #reset all grad in this network
        optimizer.zero_grad()

        #forward propogation for network
        outputs = model(inputs)

        #atain loss
        loss = criterion(outputs,labels)

        #backward propogation
        loss.backward()

        #update parameter in net
        optimizer.step()

        print("TrainingTraining Epoch:%d [%d|%d] loss:%f" % (epoch, batch_idx, len(trainloader), loss.mean()))


def val(epoch):

    total = 0
    correct = 0

    for batch_idx,(inputs,labels) in enumerate(valloader):

        inputs      = Variable(inputs.cuda())
        labels      = Variable(labels.cuda())

        outputs     = model(inputs)
        _,predicted = torch.max(outputs.data,1)

        total      += inputs.size(0)
        correct    += torch.sum(predicted == labels.data)

        #print("ValVal Epoch:%d [%d|%d] " % (epoch, batch_idx, len(valloader)))

    print("Acc: %f " % (1.0 * correct / total))




if __name__=='__main__':

    for epoch in range(num_epochs):

        train(epoch)
        val(epoch)

    # torch.save(model.state_dict(), 'checkpoints/DogCat_model.pth')












    # trainset = DogCat('./data/train', transform=transform_train)
    # valset = DogCat('./data/train', transform=transform_val)
    # trainloader = torch.utils.data.DataLoader(trainset, batch_size=opt.batchSize, shuffle=True,
    #                                           num_workers=opt.num_workers)
    # valloader = torch.utils.data.DataLoader(valset, batch_size=opt.batchSize, shuffle=False,
    #                                         num_workers=opt.num_workers)
