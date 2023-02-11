import numpy as np
import matplotlib.pyplot as plt

# This code is used to show the compounding operation
# Define the vectors
plt.arrow(0, 0, 2, 2, ec ='green', head_width = 0.05)
plt.annotate('A',xy=(1,1.2), color='green')
plt.arrow(2, 2, 2, 0, ec ='blue', head_width = 0.05)
plt.annotate('B',xy=(3,2.1), color='blue')
plt.arrow(0, 0, 4, 2, ec ='yellow', head_width = 0.05)
plt.annotate('B',xy=(2,0.80), color='yellow')
plt.xlim(0,5)
plt.ylim(0,5)
plt.grid(True)
plt.show()
