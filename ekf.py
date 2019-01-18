import numpy as np
import matplotlib.pyplot as plt



def main():
	count = 1000
	T = 1
	a = 0.01
	sit = 0
	dt = 0
	w = 10/500.0
	Z = []
	for i in range(count):
		dt = ((750-500*np.cos(sit))**2+(750-500*np.sin(sit))**2)**0.5
		sit = sit+w*T
		Z.append(dt)
	noise = np.random.normal(0, 3, count)
	Zn = Z+noise

	sit = 0
	f2sit = (5*(8*np.cos(sit)**2 + 8*np.sin(sit)**2 - (2*np.cos(sit)*(500*np.cos(sit) - 750))/125 - (2*np.sin(sit)*(500*np.sin(sit) - 750))/125))\
	/(4*((500*np.sin(sit) - 750)**2/62500 + (500*np.cos(sit) - 750)**2/62500)**(1/2)) - (5*((2*np.cos(sit)*(500*np.sin(sit) - 750))/125 - (2*np.sin(sit)*(500*np.cos(sit) - 750))/125)**2)/(8*((500*np.sin(sit) - 750)**2/62500 + (500*np.cos(sit) - 750)**2/62500)**(3/2))
	X = np.mat([0, 0]).T
	H = np.mat([1, 0])
	Ft = np.mat([[1, f2sit], [0, 1]])
	P = np.mat([[1, 0], [0, 1]])
	Q = np.mat([[2, 0], [0, 0.2]])
	R = np.mat([3])
	I = np.mat([[1, 0], [0, 1]])
	# B = np.mat([T*T/2, 0]).T

	Xt = []
	sitt = []
	for i in range(count):
		X_ = Ft*X
		P_ = Ft*P*Ft.T+Q
		K = P_*H.T*(H*P_*H.T+R).I
		X = X_ + K*(Zn[i] - H*X_)
		Xt.append(X[0, 0])
		sitt.append(X[1, 0])
		P = (I - K*H)*P_

		sit = X[1, 0]
		f2sit = (5*(8*np.cos(sit)**2 + 8*np.sin(sit)**2 - (2*np.cos(sit)*(500*np.cos(sit) - 750))/125 - (2*np.sin(sit)*(500*np.sin(sit) - 750))/125))/(4*((500*np.sin(sit) - 750)**2/62500 + (500*np.cos(sit) - 750)**2/62500)**(1/2)) - (5*((2*np.cos(sit)*(500*np.sin(sit) - 750))/125 - (2*np.sin(sit)*(500*np.cos(sit) - 750))/125)**2)/(8*((500*np.sin(sit) - 750)**2/62500 + (500*np.cos(sit) - 750)**2/62500)**(3/2))
		Ft = np.mat([[1, f2sit], [0, 1]])
	# print P
	# print noise
	plt.plot(range(count), Z, "b", label="raw")
	plt.plot(range(count), Zn, "y", label="noise")
	plt.plot(range(count), Xt, "r", label="ekf_d")
	plt.plot(range(count), sitt, "G", label="ekf_sit")
	plt.legend(loc = 0)
	plt.show()

if __name__ == '__main__':
	main()