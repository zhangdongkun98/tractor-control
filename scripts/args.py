
def generate_argparser():
    from carla_utils.utils import default_argparser
    argparser = default_argparser()

    argparser.add_argument('-d', dest='description', default='Nothing', help='[Method] description.')
    argparser.add_argument('--seed', default=0, type=int, help='seed.')
    argparser.add_argument('-m', '--method', default='None', type=str, help='[Method] Method to use.')
    argparser.add_argument('--evaluate', action='store_true', help='[Method] Eval mode (default: False)')

    argparser.add_argument('--load-model', action='store_true', help='[Model] Load model (default: False)')
    argparser.add_argument('--model-dir', default='None', type=str, help='[Model] dir contains model (default: False)')
    argparser.add_argument('--model-num', default=-1, type=str, help='[Model] model-num to use.')

    argparser.add_argument('--num-episodes', default=20000, type=int, help='number of episodes.')
    
    argparser.add_argument('--real', action='store_true', help='')
    argparser.add_argument('--visualize', action='store_true', help='')

    return argparser
