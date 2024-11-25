
import numpy as np
from driver.projection import projection

data = np.loadtxt('data/static_rtk.txt')

lat = data[:,0]
lon = data[:,1]


x, y = projection.gps2xy(lat, lon)





x1, y1 = x[:3436], y[:3436]
x2, y2 = x[3436:3436*2], y[3436:3436*2]
x3, y3 = x[3436*2:], y[3436*2:]


np.savetxt('data/statc_rtk_1.txt', np.stack([lat[:3436], lon[:3436]], axis=1), fmt='%.8f')
np.savetxt('data/statc_rtk_2.txt', np.stack([lat[3436:3436*2], lon[3436:3436*2]], axis=1), fmt='%.8f')
np.savetxt('data/statc_rtk_3.txt', np.stack([lat[3436*2:], lon[3436*2:]], axis=1), fmt='%.8f')

print(np.hypot(x - x.mean(), y - y.mean()).std())


print(np.hypot(x1 - x1.mean(), y1 - y1.mean()).std())
print(np.hypot(x2 - x2.mean(), y2 - y2.mean()).std())
print(np.hypot(x3 - x3.mean(), y3 - y3.mean()).std())



import pdb; pdb.set_trace()

