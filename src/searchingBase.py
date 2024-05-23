

class searchingBase():

    def __init__(self, pf_position, pf_laser, sf_motor) -> None:
        # Inputs expected:
        # pf_position: This variable must store the pooling function for getting the estimated position of the robot in the format [x, y]
        # pf_laser: This variable must store a array of pooling functions for each of the ultrassonic sensors.
        #           [ front_laser, right_laser, left_laser, back_laser ]
        # sf_motor: This variable must store the set function of the motor's speed

        self.sf_motor = sf_motor
        self.pf_position = pf_position
        self.pf_front_laser, self.pf_right_laser, self.pf_left_laser, self.pf_back_laser = pf_laser
        
        self.end = False

    def run(self):
        # When this function is called it will block the execution of the thread until if finds a way
        # to the base or return that it was not found
        
        while not self.end:
            # Find the nearest wall
            

            print(f'searching base algorithm {self.pf_position()}')
            self.sf_motor(2, 0)
 