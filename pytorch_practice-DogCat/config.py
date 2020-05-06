import  warnings

class DefaultConfig(object):

    enc             = 'default'

    model           = 'ResNet34'

    train_data_root = './home/ubuntu/LX/pr_practice/dataset/train'
    test_data_root  = './home/ubuntu/LX/pr_practice/dataset/test1'
    load_model_path = './checkpoints/model.pth'


    batch_size   = 128
    use_gpu      = True
    num_workers  = 4
    print_freq   = 20

    debug_file   = '/tmp/debug'
    result_file  = 'result.csv'

    max_epoch    = 10
    lr           = 0.1
    lr_decay     = 0.95

    weight_decay = 1e-4


def parse(self , kwargs):

    for k , v in kwargs.iteritems():
        if not hasattr(self,k):
            warnings.warn("Warning: opt has not attribute %s" %k)

        setattr(self , k , v)

    print('user config:')

    for k, v in self.__class__.__dict__.iteritems():
        if not k.startswith('__'):
            print(k, getattr(self, k))

DefaultConfig.parse = parse
opt = DefaultConfig()


