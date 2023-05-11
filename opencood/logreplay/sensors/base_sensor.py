"""
Base Class for sensors
"""
import cv2


class BaseSensor:
    def __init__(self, agent_id, vehicle, world, config, global_position):
        return

    def visualize_data(self):
        return

    def data_dump(self, output_folder, cur_timestamp):
        return

    def tick(self):
        return None

    def destroy(self):
        self.sensor.destroy()
        cv2.destroyAllWindows()
