
import os
import random
from PIL import  Image
from torch.utils import data
import numpy as np
from torchvision import transforms as T


class DogCat(data.Dataset):
    def __init__(self, root, transform=None, train=True, test=False):
        self.test = test
        self.train = train
        self.transform = transform
        imgs = [os.path.join(root, img) for img in os.listdir(root)]

        # test1: data/test1/8973.jpg
        # train: data/train/cat.10004.jpg

        if self.test:
            imgs = sorted(imgs, key=lambda x: int(x.split('.')[-2].split('/')[-1]))
        else:
            imgs = sorted(imgs, key=lambda x: int(x.split('.')[-2]))

        imgs_num = len(imgs)

        if self.test:
            self.imgs = imgs
        else:
            random.shuffle(imgs)
            if self.train:
                self.imgs = imgs[:int(0.7 * imgs_num)]
                print("train data is set")
            else:
                self.imgs = imgs[int(0.7 * imgs_num):]
                print("val data is set")


    def __getitem__(self, index):
        img_path = self.imgs[index]
        if self.test:
            label = 1 #if 'dog' in img_path.split('/')[-1] else 0
            #print("label: %d" % label)
        else:
            label = 1 if 'dog' in img_path.split('/')[-1] else 0
        data = Image.open(img_path)
        data = self.transform(data)
        return data, label

    def __len__(self):
        return len(self.imgs)


