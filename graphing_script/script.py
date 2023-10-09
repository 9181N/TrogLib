import numpy as np
from matplotlib import pyplot as plt

data = np.array([



(0.00, 0.00),(0.00, -20.00),(-40.00, -20.00),(-40.00, -40.00),
(0.00, 0.00),(-0.29, -2.86),(-1.12, -5.44),(-2.43, -7.79),(-4.16, -9.92),(-6.25, -11.88),
(-8.64, -13.68),(-11.27, -15.36),(-14.08, -16.96),(-17.01, -18.49),(-20.00, -20.00),
(-22.99, -21.50),(-25.92, -23.04),(-28.73, -24.63),(-31.36, -26.32),(-33.75, -28.12),
(-35.84, -30.08),(-37.57, -32.22),(-38.88, -34.56),(-39.71, -37.15),(-40.00, -40.00),

(-21.61,-20.81),


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