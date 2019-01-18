import numpy as np
import matplotlib.pyplot as plt


def main():
	count = 1000
	T = 1
	a = 0.01
	sit = 0
	xt = []
	yt = []
	sitt = []
	w = 10*2.0*3.1415926/count
	for i in range(count):
		xt_ = np.cos(sit)
		yt_ = np.sin(sit)
		sit = sit+w*T
		xt.append(xt_)
		yt.append(yt_)
		sitt.append(sit)
	noise = np.random.normal(0, 0.1, count)
	xt_no = xt+noise
	yt_no = yt+noise

	sit = 0
	f13 = -np.cos(sit)+np.sin(sit)
	f23 = -np.cos(sit)-np.sin(sit)
	X = np.mat([1, 0, 0]).T
	H = np.mat([[1, 0, 0], [0, 1, 0]])
	Ft = np.mat([[1, 0, f13], [0, 1, f23], [0, 0, 1]])
	P = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	Q = np.mat([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]])
	R = np.mat([0.1, 0.1]).T
	I = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

	xt_ekf2 = []
	yt_ekf2 = []
	sitt_ekf2 = []
	for i in range(count):
		X_ = Ft*X
		P_ = Ft*P*Ft.T+Q
		K = P_*H.T*(H*P_*H.T+R).I
		Zn = np.mat([xt_no[i], yt_no[i]]).T
		X = X_ + K*(Zn - H*X_)
		xt_ekf2.append(X[0, 0])
		yt_ekf2.append(X[1, 0])
		sitt_ekf2.append(X[2, 0])
		P = (I - K*H)*P_

		sit = X[2, 0]
		f13 = np.sin(sit)
		f23 = -np.cos(sit)
		Ft = np.mat([[1, 0, f13], [0, 1, f23], [0, 0, 1]])
	plt.plot(range(count), xt, "b", label="xt")
	plt.plot(range(count), yt, "y", label="yt")
	plt.plot(range(count), xt_no, "r", label="xt_noise")
	plt.plot(range(count), yt_no, "g", label="yt_noise")
	plt.plot(range(count), xt_ekf2, color="k", label="xt_ekf2")
	plt.plot(range(count), yt_ekf2, color="m", label="yt_ekf2")
	plt.plot(range(count), sitt, color="chartreuse", label="sitt")
	plt.plot(range(count), sitt_ekf2, color="c", label="sitt_ekf2")
	plt.legend(loc = 0)
	plt.title('kef3')

	plt.show()

if __name__ == '__main__':
	main()