import math
import random
import numpy as np
import matplotlib
import time
import matplotlib.pyplot as plt

plt.ion()

def main():
	plt_x = []
	max_len = 100
	x = 0
	while 1:
		plt_x.append(np.sin(x))
		if len(plt_x)>max_len:
			plt_x.pop(0)
			plt.clf()
			plt.plot(range(max_len), plt_x)
		else:
			plt.plot(range(len(plt_x)), plt_x)

		x = x+0.1
		plt.pause(0.01)
		# time.sleep(0.01)	
if __name__ == '__main__':
	main()