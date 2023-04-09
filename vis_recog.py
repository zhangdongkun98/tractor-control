
import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

times = np.load('./results/times.npy')
steers = np.load('./results/steers.npy')
reference_steers = np.load('./results/reference_steers.npy')


delay = times[np.where(np.abs(steers) < np.deg2rad(0.5))][-1]

print(f'DELAY = {delay}')

delta_steer = (reference_steers.mean() - 0) *0.632

lag = times[np.where( np.abs(steers - delta_steer) < np.deg2rad(0.5) )].mean() - delay
print(f'LAG = {lag}')

# import pdb; pdb.set_trace()


plt.plot(times, reference_steers, 'og')
plt.plot(times, steers, 'or')
plt.show()
