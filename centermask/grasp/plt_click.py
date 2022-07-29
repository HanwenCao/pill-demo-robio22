import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
import time


class MouseControl:
    def __init__(self, img_ui):
        self.motion_start = (0, 0)
        self.motion_end = (0, 0)
        self.img_ui = img_ui[...,::-1]
        self.motion_type = 0  # 0:pushing, 1:swiping


    def onclick(self, event):
        # Setting start location
        self.motion_start = (event.xdata, event.ydata)
        print("Start:  button=%d, x=%d, y=%d, xdata=%f, ydata=%f" % (
            event.button, event.x, event.y, event.xdata, event.ydata))
        if event.button == 2:
            # right click: pushing
            self.motion_type = 0
        elif event.button == 3:
            # left click: swiping
            self.motion_type = 1
        else:
            print("Unknown motion type.")


    def onrelease(self, event):
        # Setting end location
        self.motion_end = (event.xdata, event.ydata)
        print("End:  button=%d, x=%d, y=%d, xdata=%f, ydata=%f" % (
            event.button, event.x, event.y, event.xdata, event.ydata))

        time.sleep(1)
        self.exit()

    def click_detect(self):
        ax = plt.imshow(self.img_ui)
        fig = ax.get_figure()
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        cid = fig.canvas.mpl_connect('button_release_event', self.onrelease)
        plt.show()

    def exit(self):
        # fig.canvas.mpl_disconnect(cid)
        plt.close()

    def __del__(self):
        # fig.canvas.mpl_disconnect(cid)
        plt.close()

if __name__ == "__main__":
    ms = MouseControl(plt.imread("test2.jpg"))
    ms.click_detect()







# def bool_reserve(b):
#     return not b
#
#
# def main():
#
#     motion_start = (0, 0)
#     motion_end = (0, 0)
#     mouse_flag = True  # True for setting motion starting
#
#     def onclick(event):
#         # Event Callback
#         print("button=%d, x=%d, y=%d, xdata=%f, ydata=%f" % (
#             event.button, event.x, event.y, event.xdata, event.ydata))
#
#     img_ui = plt.imread("test2.jpg")
#     ax = plt.imshow(img_ui)
#     fig = ax.get_figure()
#
#     cid = fig.canvas.mpl_connect('button_press_event', onclick)
#
#     plt.show()
#
# if __name__ == "__main__":
#     main()