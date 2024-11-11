import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({'font.size': 13})

folder_name = "2023-12-18-15-56-37/"

des_trajectory = np.genfromtxt(folder_name+"file_des_trajectory.csv", delimiter = ',')
current_state = np.genfromtxt(folder_name+"file_current_state.csv", delimiter = ',')
current_control = np.genfromtxt(folder_name+"file_current_control.csv", delimiter = ',')
reference_TCP = np.genfromtxt(folder_name+"file_reference_TCP.csv", delimiter = ',')
actual_TCP = np.genfromtxt(folder_name+"file_actual_TCP.csv", delimiter = ',')




# ctrl_period = np.genfromtxt(timestamp + "/dt.csv", delimiter = ',')


#print("q", q.shape)
#print("dq", dq.shape)
#print("ref_q", ref_q.shape)
#print("ref_dq",ref_dq.shape)
#print("x_l", x_l.shape)
#print("x_r", x_r.shape)
#print("ref_x_l", ref_x_l.shape)
#print("ref_x_r", ref_x_r.shape)
#print("ref_dx_l", ref_dx_l.shape)
#print("ref_dx_r", ref_dx_r.shape)

#print("torques", torques.shape)

# plt.figure()
# plt.plot(ctrl_period[1:])
# plt.grid()
# plt.ylabel("dt [s]")
# plt.show()

n_steps = des_trajectory.shape[1]
dt = 0.06667
time = dt * np.arange(0, n_steps)

T_start = 0.0
T_end = time[-1]
n_start = int(n_steps * T_start/time[-1])
n_end = int(n_steps * T_end/time[-1])



fig, axs = plt.subplots(3, 1)
axs = axs.reshape(-1)
for i in range(3):
  axs[i].plot(time[n_start:n_end], des_trajectory[i,n_start:n_end], 'tab:red', label = 'Desired object trajectory')
  axs[i].grid()
  axs[i].legend()
 
for i in range(3):
  axs[i].plot(time[n_start:n_end], current_state[i,n_start:n_end], 'tab:blue', label = 'Actual object trajectory')
  axs[i].grid()
  axs[i].legend()
fig.suptitle(r'Object trajectory')
plt.show()

error = des_trajectory[0:3,n_start:n_end] - current_state[0:3,n_start:n_end]
print('shape of array :', error.shape) 
fig, axs = plt.subplots(3, 1)
axs = axs.reshape(-1)
for i in range(3):
  axs[i].plot(time[n_start:n_end], error[i,n_start:n_end], 'tab:red', label = '')
  axs[i].grid()
  axs[i].legend()
fig.suptitle(r'Error')
plt.show()


fig, axs = plt.subplots(5, 1)
axs = axs.reshape(-1)
for i in range(5):
  axs[i].plot(time[n_start:n_end], current_state[i+3,n_start:n_end], 'tab:blue', label = '')
  axs[i].grid()
  axs[i].legend()
fig.suptitle(r'MPC state')
plt.show()


fig, axs = plt.subplots(5, 1)
axs = axs.reshape(-1)
for i in range(5):
  axs[i].plot(time[n_start:n_end], current_control[i,n_start:n_end], 'tab:blue', label = '')
  axs[i].grid()
  axs[i].legend()
fig.suptitle(r'MPC control')
plt.show()

fig, axs = plt.subplots(7, 1)
axs = axs.reshape(-1)
for i in range(7):
  axs[i].plot(time[n_start:n_end], reference_TCP[i,n_start:n_end], 'tab:red', label = 'reference TCP')
  axs[i].grid()
  axs[i].legend()

for i in range(7):
  axs[i].plot(time[n_start:n_end], actual_TCP[i,n_start:n_end], 'tab:blue', label = 'actual TCP')
  axs[i].grid()
  axs[i].legend()
fig.suptitle(r'TCP')
plt.show()




