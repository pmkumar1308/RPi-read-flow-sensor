import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# file_name = input("Enter file name suffix (after Rpi_data_readings_cyclic_flow_rate): ")
file_path = "./Data_from_PropValves/Rpi_data_readings_cyclic_flow_rate_def2.csv"

header_names = ['Time', 'Actual Value' , 'Set Value', 'Pressure', 'Mass']
df = pd.read_csv(file_path, names = header_names)


# plt.figure()
df["Time"] = pd.Series(list(range(len(df))))
df.plot(x="Time", y=header_names[1:-1], label = ["Actual Value(l/min)", "Set Value(l/min)", "Pressure"])
# df.plot(x="Time", y="Pressure", c = "r")
df.plot(x="Time", y="Mass")

plt.show()