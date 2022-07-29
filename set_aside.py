from auboi5_controller import AuboController
import time

if __name__ == '__main__':
    auboi5_controller = AuboController('192.168.1.115')
    
    start = [0.1,-0.3]
    end = [0.3,-0.5]
    auboi5_controller.set_aside(start,end)

    start = [0.1,-0.3]
    end = [0.0,-0.4]
    auboi5_controller.set_aside(start,end)
    # pose1 = [0.1678063935219885, -0.3170875217172737, 0.5329802955066322,0.12200129680275658, -0.9924559092858365, 0.01183003353330129, -0.0026461308868564923]
    # auboi5_controller.moveL(pose1)    

    auboi5_controller.neg_srt(0)