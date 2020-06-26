"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

Classes State and States from PythonRobotics

"""
import matplotlib.pyplot as plt
import time

from utils import *


if __name__ == "__main__":

    total_time = 20 # max simulation time in seconds
    simulation_time = 0 #variable to increment during the simulation
    dt = 1e-3 # delta time

    bike = State(dt=dt) # instance the bike model
    states = States() # instance to save the bike states

    #start the simulation
    while simulation_time < total_time:

    
        v = 2 * step(simulation_time, 0) # Velocity in [m/s]
        delta = 0.13 * step(simulation_time, 0) # Steering anle [rad] / Anguloda da direção
        bike.update(v, delta, simulation_time)
        states.append(simulation_time, bike)

        simulation_time += dt        
    #End of the simulation

    print('Tempo total de simulação: %i s' % simulation_time)
    print('Ultima posicao: x: %.2f m ; y: %.2f m' % (bike.x, bike.y))
    print('Numero de amostras da simulacao: %i' % len(states.y))
    
    plt.axis("equal")
    plt.grid(True)
    colors = cm.Reds(np.linspace(0, 1, len(states.x)))
    plt.scatter(states.x, states.y, color=colors, s=2)
    plt.show()

