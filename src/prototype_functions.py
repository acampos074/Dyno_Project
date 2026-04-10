import numpy as np
from numpy.fft import fft
from scipy.linalg import expm
from scipy.signal import chirp
import matplotlib.pyplot as plt

# System parameters RL Circuit
R = 0.362 # Ohms
L = 567e-6 # Henries

num_states = 1

A = np.array([-R/L])

B = np.array([1/L])

C = np.array([1])


# Simulation parameters
Fs = 40000
h = 1/Fs  # Time step (s)
T = 10  # Total time
N = int(T/h)
#N = 400
#T = N*h

Ad = expm(h*A)
Bd = np.dot((1/A),(Ad-np.eye(num_states)))
Bd = np.dot(Bd,B)
Cd = C

t = np.arange(0,T,h)
X = np.zeros(N+1)

# Generate the chirp signal
t1 = T  # End time for the chirp
f0 = 0  # Start frequency
f1 = 100  # End frequency
Ud = chirp(t, f0, t1, f1, method='linear', phi=0)

# Reshape the input signal
U = 0.2*np.vstack(Ud)

# Simulation state space representation
for i in range(1, N):
    X[i+1] = np.dot(Ad,X[i]) + np.dot(Bd,U[i])

current = X[0:N] # output
voltage = U # input

#np.arange(0,10,1)

Fn = Fs/2
inprm = voltage-np.mean(voltage)*0
outrm = current-np.mean(current)*0
FTinpr = np.fft.fft(inprm)                                     # Fourier Transform Of Mean-Corrected Data
FToutr = np.fft.fft(outrm)                                    # Fourier Transform Of Mean-Corrected Data
Fv = np.arange(0,1,1/(np.fix(N/2)+1))*Fn                           # Frequency Vector (One-Sided)
Iv = np.arange(1,np.size(Fv)+1,1)                        # Index Vector

H = np.zeros(np.size(Iv))
for i in range(1, np.size(Iv)):
    H[i] = abs(FToutr[i]/FTinpr[i])
    
fig, ax = plt.subplots()
ax.plot(t,outrm,label='Continous',color='blue',linestyle='-')
ax.set_xlabel('Time [s]')
ax.set_ylabel('i(t) [A]')
ax.legend()  # Add a legend.
ax.set_aspect('auto',adjustable='box')
plt.grid(True)
#plt.ylim(0,0.6)
#plt.xlim(0,100)
plt.show()


fig, ax = plt.subplots()
ax.plot(t,inprm,label='Continous',color='blue',linestyle='-')
ax.set_xlabel('Time [s]')
ax.set_ylabel('v(t) [V]')
ax.legend()  # Add a legend.
ax.set_aspect('auto',adjustable='box')
plt.grid(True)
#plt.ylim(0,0.6)
#plt.xlim(0,0.008)
plt.show()

fig, ax = plt.subplots()
ax.semilogx(Fv,20*np.log10(H),label='Continous',color='blue',linestyle='-')
ax.set_xlabel('Frequency [Hz]')
ax.set_ylabel('Transfer Function [dB]')
ax.legend()  # Add a legend.
ax.set_aspect('auto',adjustable='box')
plt.grid(True)
#plt.ylim(0,0.6)
plt.xlim(0,100)
plt.show()