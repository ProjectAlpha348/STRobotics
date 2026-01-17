#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RPLCD.i2c import CharLCD

class DisplayNode(Node):
    def __init__(self):
        super().__init__("display_node")
        self.lcd_ = None
        self.subscriber_ = self.create_subscription(String, "display", self.callback_Display, 10)
        try:
            self.lcd_ = CharLCD(
                i2c_expander='PCF8574',
                address=0x27,
                port=1,
                cols=20,
                rows=4,
                charmap='A00',
                auto_linebreaks=False
            )
        except KeyboardInterrupt:
            self.lcd_.clear()
        self.lcd_.clear()
        self.Rows_: int = 0
        self.lcd_.cursor_pos = (0,0)
        self.display: list = []
        self.display.append("STRobotics is ready")
        self.display.append("")
        self.display.append("")
        self.display.append("")
        self.lcd_.write_string("STRobotics is ready")
        self.get_logger().info("Display_Node has been started.")
       

    def callback_Display(self, msg: String):
        self.get_logger().info(msg.data)
        Testo = msg.data
        if Testo == "D01":
            self.lcd_.clear()
            self.Rows_ = 0
            self.lcd_.cursor_pos = (0,0)
        else:self.WriteDisplay(Testo)

    def WriteDisplay(self,msg:str):
        message = msg
        if self.Rows_== 3:
            self.display[0]=self.display[1]
            self.display[1]=self.display[2]
            self.display[2]=self.display[3]
            self.display[3]=message
            for row in range(4):
                self.lcd_.cursor_pos=(row,0)
                self.lcd_.write_string(self.display[row])
        else:
            self.Rows_ += 1
            self.lcd_.cursor_pos=(self.Rows_,0)
            self.display[self.Rows_] = message
            self.lcd_.write_string(self.display[self.Rows_])
            

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()