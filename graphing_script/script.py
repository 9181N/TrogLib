import numpy as np
from matplotlib import pyplot as plt

data = np.array([



(0.00,-2.00),(0.02,-2.00),(0.04,-2.00),(0.06,-2.00),(0.08,-2.00),(0.10,-2.00),
(0.12,-2.00),(0.14,-2.00),(0.16,-2.00),(0.18,-2.00),(0.20,-2.00),
(0.22,-2.00),(0.24,-2.00),(0.26,-2.00),(0.28,-2.00),(0.30,-2.00),
(0.32,-2.00),(0.34,-2.00),(0.36,-2.00),(0.38,-2.00),(0.40,-2.00),
(0.42,-2.00),(0.44,-2.00),(0.46,-2.00),(0.48,-2.00),(0.50,-2),
(0.52,-2.00),(0.54,-2.00),(0.56,-2.00),(0.58,-2.00),(0.60,-2.00),
(0.62,-2.00),(0.64,-2.00),(0.66,-2.00),(0.68,-2.00),(0.70,-2.00),
(0.72,-2.00),(0.74,-2.00),(0.76,-2.00),(0.78,-2.00),(0.80,-2.00),
(0.82,-2.00),(0.84,-2.00),(0.86,-2.00),(0.88,-2.00),(0.90,-2.00),
(0.92,-2.00),(0.94,-2.00),(0.96,-2.00),(0.98,-2.00),(1.00,-2.00),
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