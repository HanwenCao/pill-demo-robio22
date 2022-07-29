from auboi5_controller import AuboController
import time

if __name__ == '__main__':
    auboi5_controller = AuboController('192.168.1.115')
    
    while 1:
        auboi5_controller.getCurrentWaypoint()
        time.sleep(0.5)
        

