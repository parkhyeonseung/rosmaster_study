from ardu_serial import Ardu
## v hover_yolo_joy_mode4_8_31
class hover_app():
    def __init__(self):
        self.hover = Ardu()

    
    def get_xy(self,x,y):  ## cmd =10
        if x>=0:
            x_data = ( int(x*300) // 10) * 10
            y_data = ( int(y*300)  //10) * -10
        else:
            x_data = ( int(x*300) // 10) * 10
            y_data = ( int(y*300)  //10) * 10

        x_hover_data = 'g'+str(x_data)
        y_hover_data = 't'+str(y_data)
        print(x_hover_data)
        print(y_hover_data)
        self.hover.input(x_hover_data)
        self.hover.input(y_hover_data)
        
    def all_stop(self): ## cmd =20
        self.hover.input('00')
        
    def close(self):
        self.hover.close()