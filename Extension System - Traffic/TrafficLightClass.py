class TrafficLightClass:
    def __init__(self, position, green_duration, orange_duration, red_duration):
        self.position = position
        self.green_duration = green_duration
        self.orange_duration = 0#orange_duration
        self.red_duration = red_duration

    def status(self, time):
        
        # Total cycle time
        cycle_time = self.green_duration + self.orange_duration + self.red_duration
        
        # Current time in the cycle
        phase = time % cycle_time

        # Determine if the light is green, orange, or red
        if phase < self.green_duration:
            return "green"
        elif phase < self.green_duration + self.orange_duration:
            return "orange"
        else:
            return "red"
