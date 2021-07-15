from car_model import simulate_car_model

class ChangeLaneEnv():
    def __init__(self):
        self.lane_number=3
        self.lane_length=2
        self._car_list=[]


    def draw_env(self):
        for car in self._car_list:
            car.draw_car()








if __name__ == "__main__":
    env=ChangeLaneEnv()