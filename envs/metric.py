
import numpy as np



def calculate_metric(error_paths, metric=0.025):
    ratio_path = (np.where(error_paths <= metric, 1, 0).sum() / error_paths.shape[0]) *100
    metric_str = f'error mean: {np.round(np.mean(error_paths), 4)}, ratio: {np.round(ratio_path, 4)} %'
    print(metric_str)
    return metric_str, np.round(ratio_path, 4)


