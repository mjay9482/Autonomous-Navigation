#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 22:46:00 2024

@author: mrityunjay
"""

import os
import numpy as np
import matplotlib.pyplot as plt
#hi = os.path.join(os.getcwd() , f'hello')
import os
x = np.array([1,2])
current_directory = os.getcwd()
print(current_directory)
hi = os.path.join(os.getcwd(), f'hello')
fig, ax = plt.subplots()
ax.scatter(x[0], x[1])
# Save the plot inside the 'hello' directory
plot_path = os.path.join(hi, 'scatter_plot.png')
plt.savefig(plot_path)

# Show the plot
plt.show()