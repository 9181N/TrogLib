import numpy as np
from matplotlib import pyplot as plt

data = np.array([
    (20,30),(40,0),(0,0),(0,0),(50,0),(70,0),(0,0),(0,0),(0,0),(0,0),
])
x, y = data.T
plt.plot(x, y, label = "Line 1", color='gray', linestyle='none', linewidth = 3,
         marker='.', markerfacecolor='blue', markersize=3)

#plt.xlim(-20,20)
#plt.ylim(-20,20)

plt.show()



#example graph settings
#plt.plot(x,y)
#plt.plot(x, y, label = "Line 1", color='gray', linestyle='solid', linewidth = 3,
#         marker='o', markerfacecolor='black', markersize=3)
#plt.plot(x, y, label = "Line 1", color='gray', linestyle='solid', linewidth = 3,
#         marker='o', markerfacecolor='black', markersize=3)