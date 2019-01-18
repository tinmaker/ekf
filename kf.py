import numpy as np
import matplotlib.pyplot as plt



def main():
	count = 1000
	T = 1
	a = 0.01
	vt = 0
	st = 0
	Z = []
	for i in range(count):
		Z.append(st)
		st = (st+vt*T+0.5*a*T**2)
		vt = vt+a*T
	noise = np.random.normal(0, 200, count)
	Zn = Z+noise

	X = np.mat([0, 0]).T
	P = np.mat([[1, 0], [0, 1]])
	F = np.mat([[1, T], [0, 1]])
	Q = np.mat([[0.000001, 0], [0, 0.000001]])
	H = np.mat([1, 0])
	R = np.mat([200])
	I = np.mat([[1, 0], [0, 1]])
	B = np.mat([T*T/2, 0]).T

	Xt = []
	for i in range(count):
		X_ = F*X+B*a
		P_ = F*P*F.T+Q
		K = P_*H.T*(H*P_*H.T+R).I
		X = X_ + K*(Zn[i] - H*X_)
		Xt.append(X[0, 0])
		P = (I - K*H)*P_
	print P
	# print noise
	plt.plot(range(count), Z, "b", label="raw")
	plt.plot(range(count), Zn, "y", label="noise")
	plt.plot(range(count), Xt, "r", label="kf")
	plt.legend(loc = 0)
	plt.show()

if __name__ == '__main__':
	main()