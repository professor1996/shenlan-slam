

import torch
import torch.utils.data as Data


BATCH_SIZE = 5


x = torch.linspace(1  , 10 , 10)
y = torch.linspace(10 , 1  , 10)


torch_dataset = Data.TensorDataset(data_tensor=x , target_tensor=y)

loader = Data.DataLoader(
    dataset = torch_dataset,
    batch_size = BATCH_SIZE,
    shuffle=True,
    num_workers=2,  #threads
)


for epoch in range(10):

    for step , (batch_x , batch_y) in enumerate(loader):

        #traing ....
        print('Epoch: ', epoch , '| Step: ' , step , '| batch_x:' , batch_x.numpy() , '| batch_y:' , batch_y.numpy())

