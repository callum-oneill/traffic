import matplotlib.pyplot as plt
import simulationFunctions as sf
import multiprocessing as mp
import statsmodels.api as sm
import matplotlib.animation as animation
import matplotlib.cm as cm
import TrafficVisualization as tv
import numpy as np

def plot_best_fit_lowess(ax, x, y, xlabel, ylabel, title, color):
    ax.scatter(x, y, color='black', marker='x', label='Data Points', alpha=0.25)

    # Apply LOWESS smoothing
    lowess = sm.nonparametric.lowess(y, x)  
    x_smooth, y_smooth = zip(*lowess)

    ax.plot(x_smooth, y_smooth, color=color, linestyle='-', label='LOWESS Fit')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid()

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length):
    
    # Run simulation
    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)
    
    # Compute average velocity (glob_avg_velocity, loc_avg_velocity)
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    # Return only the values that are needed
    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity

if __name__ == "__main__":
    
    # Model parameters
    max_cars = 10
    road_length = 300
    steps = 1000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 2
    end_cars = max_cars
    step_cars = 2

    # List of car counts to simulate
    car_counts = range(start_cars, end_cars + step_cars, step_cars)
    cars = []

    # Parallel processing using multiprocessing Pool
    with mp.Pool(processes=mp.cpu_count()) as pool:
        results = pool.starmap(run_simulation, [(N, time_step, steps, steps_before_measure, detection_point, road_length) for N in car_counts])

    # Unpack results
    global_flow, local_flow = [], []
    global_density, local_density = [], []
    global_average_velocity, local_average_velocity = [], []

    for res in results:
        N, carsMaxN, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity = res

        global_flow.append(glob_flow)
        global_density.append(glob_density)
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        global_average_velocity.append(glob_avg_velocity)
        local_average_velocity.append(loc_avg_velocity)
        cars = carsMaxN

    # Create visualisation
    #tv.TrafficVisualization(cars, road_length, road_length / (2 * np.pi), 5, 250, 5)

    # Create a 2x3 figure layout
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    # Plot 1: Global Flow vs. Global Density
    axes[0, 0].plot(global_density, global_flow, 'bo-', label='Global Flow')
    axes[0, 0].set_xlabel('Density (cars/km)')
    axes[0, 0].set_ylabel('Flow (cars/hour)')
    axes[0, 0].legend()
    axes[0, 0].grid()

    # Plot 2: Local Flow vs. Local Density
    axes[0, 1].plot(local_density, local_flow, 'ro-', label='Local Flow')
    axes[0, 1].set_xlabel('Density (cars/km)')
    axes[0, 1].set_ylabel('Flow (cars/hour)')
    axes[0, 1].legend()
    axes[0, 1].grid()

    # Plot 3: Global Avg Velocity vs. Density
    axes[0, 2].plot(global_density, global_average_velocity, 'bo-', label='Global Avg Velocity')
    axes[0, 2].set_xlabel('Density (cars/km)')
    axes[0, 2].set_ylabel('Average Velocity (m/s)')
    axes[0, 2].legend()
    axes[0, 2].grid()

    # Plot 4: Local Avg Velocity vs. Density
    axes[1, 0].plot(local_density, local_average_velocity, 'ro-', label='Local Avg Velocity')
    axes[1, 0].set_xlabel('Density (cars/km)')
    axes[1, 0].set_ylabel('Average Velocity (m/s)')
    axes[1, 0].legend()
    axes[1, 0].grid()

    # Plot 5: Global Flow vs. Global Velocity
    axes[1, 1].plot(global_flow, global_average_velocity, 'bo-', label='Global Flow')
    axes[1, 1].set_xlabel('Flow (cars/hour)')
    axes[1, 1].set_ylabel('Average Velocity (m/s)')
    axes[1, 1].legend()
    axes[1, 1].grid()

    # Plot 6: Local Flow vs. Local Velocity
    axes[1, 2].plot(local_flow, local_average_velocity, 'ro-', label='Local Flow')
    axes[1, 2].set_xlabel('Flow (cars/hour)')
    axes[1, 2].set_ylabel('Average Velocity (m/s)')
    axes[1, 2].legend()
    axes[1, 2].grid()

    # Adjust layout to prevent overlap
    plt.tight_layout()
    plt.show()

    '''
    # Create a figure with multiple subplots
    fig, axes = plt.subplots(3, 2, figsize=(12, 12))

    # Plot each graph in a subplot
    plot_best_fit_lowess(axes[0, 0], global_density, global_flow, 'Density (cars/km)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
    plot_best_fit_lowess(axes[0, 1], global_flow, global_average_velocity, 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Flow vs. Velocity (Global)', 'blue')
    plot_best_fit_lowess(axes[1, 0], local_density, local_flow, 'Density (cars/km)', 'Flow (cars/hour)', 'Local Flow vs. Density', 'red')
    plot_best_fit_lowess(axes[1, 1], global_density, global_average_velocity, 'Density (cars/km)', 'Average Velocity (m/s)', 'Global Avg Velocity vs. Density', 'blue')
    plot_best_fit_lowess(axes[2, 0], local_density, local_average_velocity, 'Density (cars/km)', 'Average Velocity (m/s)', 'Local Avg Velocity vs. Density', 'red')

    # Hide empty subplot
    axes[2, 1].axis('off')

    # Adjust layout and show all plots in one window
    plt.tight_layout()
    plt.show()
    '''