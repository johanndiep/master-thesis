import serial
import matplotlib.pyplot as plt

class Serial():
	def __init__(self):
		super(Serial, self).__init__()

		for i in range(0, 10):
			try:
				self.ser = serial.Serial("/dev/ttyACM" + str(i))
				print("Connected to serial /dev/ttyACM" + str(i))
				break

			except:
				pass

	def get_histogram(self):
		list = []
		j = 0

		while j < 1000:
			current_line = str(self.ser.readline())
			if "distance" in current_line:
				list.append(int(current_line[15:19]))

				print(j)
				j = j + 1

		plt.hist(list, bins = 30, histtype = "bar", ec = "black")
		plt.show()


def main():
	obj = Serial()
	obj.get_histogram()


if __name__ == '__main__':
	main()