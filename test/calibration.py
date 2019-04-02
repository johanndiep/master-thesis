import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn import linear_model

Data = pd.read_csv("measurements_2.csv").values

X_data = Data[:, 0]
Y_data = Data[:, 1]

x = np.arange(3)

plt.plot(X_data, Y_data, "ro")
plt.plot(x, x)

X_data_regression = X_data[:25].reshape(-1, 1)
Y_data_regression = Y_data[:25].reshape(-1, 1)

#print(X_data_regression)
#print(Y_data_regression)

regression = linear_model.LinearRegression()
regression.fit(X_data_regression, Y_data_regression)

Y_data_predict = regression.predict(X_data_regression)
plt.plot(X_data_regression, Y_data_predict, color = "green")

plt.show()

print(regression.intercept_)