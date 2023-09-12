from numpy import array
import numpy as np
from vizulize import run_localization



if __name__ == "__main__":
    landmarks = array([[5, 10], [10, 5], [15, 15]])

    ekf = run_localization(
        landmarks, std_vel=0.1, std_steer=np.radians(1),
        std_range=0.3, std_bearing=0.1)
    print('Final P:', ekf.P.diagonal())