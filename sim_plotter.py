import numpy as np
import matplotlib.pyplot as plt

# Load data, ignore non-numeric lines safely
data = np.genfromtxt(
    "sim_out.txt",
    comments=None,
    skip_header=3,
    invalid_raise=False
)

# Remove rows with NaNs (e.g. "Done.")
data = data[~np.isnan(data).any(axis=1)]

t     = data[:,0]
x     = data[:,1]
y     = data[:,2]
yaw   = data[:,3]
v     = data[:,4]
steer = data[:,5]

plt.figure(figsize=(14,4))

plt.subplot(1,3,1)
plt.plot(x, y)
plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Vehicle trajectory")

plt.subplot(1,3,2)
plt.plot(t, v)
plt.xlabel("time [s]")
plt.ylabel("v [m/s]")
plt.title("Speed")

plt.subplot(1,3,3)
plt.plot(t, steer)
plt.xlabel("time [s]")
plt.ylabel("steer [deg]")
plt.title("Steering command")

plt.tight_layout()
plt.show()
