class PreCreateCar():
    def __init__(self, create_time, alive_time, lane, strategy,car_model):
        self.create_time=create_time
        self.strategy=strategy
        self.alive_time=alive_time
        self.car_model=car_model
        self.lane=lane
