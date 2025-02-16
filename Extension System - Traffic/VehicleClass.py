import numpy as np

class VehicleClass:

    def __init__(self, car_id, lane, pos, vel, acc, headway, dv, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

        self.car_id = car_id

        self.lane = lane
        self.pos = pos

        self.vel = vel
        self.acc = acc

        self.headway = headway
        self.dv = dv
        
        self.speedlim = speedlim
        self.des_speed = speedlim 
        self.des_speed_inv = 1.0 / speedlim if speedlim > 0 else 0

        self.acc_exp = acc_exp
        self.time_gap = time_gap
        self.min_gap = min_gap

        self.comf_decel = comf_decel
        self.acc_max = acc_max
        self.length = length

    def upd_pos_vel(cars, time_step, L, traffic_light, light_status):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))
        maxStoppingDistance = np.zeros(len(cars))

        for i, car in enumerate(cars):
            if car.car_id != -1:
                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

                # Calculate acceleration using IDM
                acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)

                maxStoppingDistance[i] = -car.des_speed**2 / (2 * car.comf_decel)

        for i, car in enumerate(cars):
            if car.car_id != -1:

                # Update velocity and position
                velnew[i] = car.vel[-1] + acc_new[i] * time_step
                posnew[i] = car.pos[-1] + car.vel[-1] * time_step + 0.5 * acc_new[i] * time_step**2

                # Ensure velocity does not go negative
                if velnew[i] < 0:

                    # Calculate time to stop
                    if abs(acc_new[i]) > 1e-6:
                        t_stop = -car.vel[-1] / acc_new[i]
                    else:
                        t_stop = 0

                    # Update position and velocity to stop at t_stop
                    posnew[i] = car.pos[-1] + car.vel[-1] * t_stop + 0.5 * acc_new[i] * t_stop**2
                    velnew[i] = 0
                
        for i, car in enumerate(cars): 
            if car.car_id != -1:
                
                # Handle stopping at red light
                if light_status == 'red' and car.pos[-1] == traffic_light.position:
                    posnew[i] = car.pos[-1]
                    velnew[i] = 0
                    acc_new[i] = 0

                # Check if car should stop at the red light
                elif light_status == "red" and car.pos[-1] < traffic_light.position <= posnew[i]:
                    
                    index = (i + 1) % len(cars)
                    next_car = cars[index]

                    if next_car.car_id == -1:
                        index = (i + 2) % len(cars)
                        next_car = cars[index]

                    if next_car.vel[-1] == 0 or next_car.pos[-1] == traffic_light.position or velnew[index] == 0:
                        safe_gap = car.min_gap + next_car.length
                        min_pos = (next_car.pos[-1] - safe_gap) % L

                        if car.pos[-1] < min_pos <= posnew[i]:
                            velnew[i] = 0
                            posnew[i] = min_pos
                            acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)
                    else:
                        velnew[i] = 0
                        posnew[i] = traffic_light.position
                        distance = traffic_light.position - car.pos[-1]
                        acc_new[i] = -car.vel[-1]**2 / (2 * distance)

                elif light_status == 'red':
                    index = (i + 1) % len(cars)
                    next_car = cars[index]

                    if next_car.car_id == -1:
                        index = (i + 2) % len(cars)
                        next_car = cars[index]

                    if next_car.vel[-1] == 0 or next_car.pos[-1] == traffic_light.position or velnew[index] == 0:
                        safe_gap = car.min_gap + next_car.length
                        min_pos = (next_car.pos[-1] - safe_gap) % L

                        if car.pos[-1] < min_pos <= posnew[i]:
                            velnew[i] = 0
                            posnew[i] = min_pos
                            acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)
                elif light_status == "orange":

                    # Calculate remaining time in the orange phase
                    cycle_time = traffic_light.green_duration + traffic_light.orange_duration + traffic_light.red_duration
                    time_in_cycle = current_time % cycle_time
                    time_left_in_orange = traffic_light.orange_duration - time_in_cycle if time_in_cycle < traffic_light.green_duration + traffic_light.orange_duration else 0
                    
                    distance_to_light = traffic_light.position - car.pos[-1]
                    # Calculate time required to reach the light at the current velocity
                    if velnew[i] > 0:
                        time_to_light = distance_to_light / velnew[i]
                    else:
                        time_to_light = float('inf')

                    # If the car can reach the light before it turns red, continue
                    if time_to_light > time_left_in_orange:
    
                        # Slow down to rest if it cannot make it in time
                        velnew[i] = 0
                        posnew[i] = car.pos[-1] + velnew[i] * time_step
                        acc_new[i] = -car.comf_decel
                
                '''
                elif light_status == 'orange':
                # Check if the car has enough time to pass through the light before it turns red
                time_to_light = dist_to_light / car.vel[-1] if car.vel[-1] > 0 else float('inf')
                if time_to_light >= time_step:

                    # The car needs to stop, so check if it can stop in time
                    if dist_to_light < stopping_distance:
                        # If the car is too close, apply hard deceleration to stop before hitting the light
                        acc_new[i] = -car.vel[-1]**2 / (2 * dist_to_light)
                        velnew[i] = 0  
                    else:
                        # If there's enough distance, apply comfortable deceleration to stop
                        acc_new[i] = -car.vel[-1]**2 / (2 * dist_to_light)
                        velnew[i] = 0
                '''

        # Set new position and velocity values
        for i, car in enumerate(cars):
            car.acc.append(acc_new[i])
            car.pos.append(posnew[i] % L)
            car.vel.append(velnew[i])

        return cars


    def update_cars(cars, N, L, time_step):

        while True:
            changes = False

            # Loop backward to adjust positions for safety gaps
            for i in range(N - 1, -1, -1):
                if cars[i].car_id == -1:
                    continue

                car = cars[i]
                next_car = cars[(i + 1) % N]
                displacement = 0 

                # Get minimum safety gap
                min_safe_gap = car.min_gap + next_car.length

                # Compute headway
                if next_car.pos[-1] > car.pos[-1]:
                    car.headway.append(next_car.pos[-1] - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] + L - car.pos[-1])

                if car.headway[-1] < min_safe_gap:
                    
                    changes = True
                    car.headway[-1] = min_safe_gap

                    # Adjust the position of the current car to maintain the minimum gap
                    car.pos[-1] = (next_car.pos[-1] - min_safe_gap) % L

                    displacement = car.pos[-1] - car.pos[-2]

                    if displacement < 0:
                        displacement += L

                    car.acc[-1] = 2 * (displacement - car.vel[-2] * time_step) / time_step**2
                    car.vel[-1] = 2 * displacement / time_step - car.vel[-2]
            
            if not changes:
                break

        for  i, car in enumerate(cars):

            next_car = cars[(i + 1) % N]

            car.dv.append(next_car.vel[-1] - car.vel[-1])
        
        return cars