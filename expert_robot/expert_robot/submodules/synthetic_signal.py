import numpy as np
import matplotlib.pyplot as plt

# Parameters for defining the synthetic user signal:
amplitude = 1.5
duration = 5.0 # For step signal at max amplitude
period = 20.0 - duration # Seconds
freq = 1.0 / period

# Generate a combination of a sine and step signal:
t = np.linspace(0, period, int(50*period))

# Generate the sine signal:
sin_signal = amplitude * np.sin(2 * np.pi * freq * t)

# Generate the step signal:
step_signal = np.zeros(int(duration*50))

# For half of the duration, the signal is at max amplitude:
step_signal[:int(50*duration/2)] = amplitude

# For the other half of the duration, the signal is at min amplitude:
step_signal[int(50*duration/2):] = -amplitude

# Find timestamps for peak amplitudes
t_max = t[np.argmax(sin_signal)]
t_min = t[np.argmin(sin_signal)]

print(f"t_max: {t_max}")
print(f"t_min: {t_min}")

# Insert the first half of the step signal at the peak amplitude:
signal = np.concatenate((sin_signal[:int(50*t_max)], step_signal[:int(50*duration/2)]))
signal = np.concatenate((signal, sin_signal[int(50*t_max):int(50*t_min)]))
signal = np.concatenate((signal, step_signal[int(50*duration/2):]))
signal = np.concatenate((signal, sin_signal[int(50*t_min):]))

t = np.linspace(0, 20, 50*20)

# Plot the synthetic signal:
plt.plot(t, signal)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Synthetic User Signal')

# Save the output of the synthetic signal to a file:
np.savetxt('/home/r01_ros2_ws/src/expert_robot/expert_robot/expert_robot/submodules/synthetic_signal.txt', signal)

plt.show()