import rclpy
from rclpy.node import Node
import time
from custom_interfaces.srv import GetSpectrum  
import threading
import serial
from nanospec.NanoLambdaNSP32 import *

class SpectrumService(Node):
    def __init__(self):
        super().__init__('NSP32_service_node')
        self.wavelengths = []
        self.spectrum = []
        self.ser = serial.Serial('/dev/nanospec', baudrate=115200, bytesize=serial.EIGHTBITS, 
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.nsp32 = NSP32(self.data_channel_send_data, self.on_return_packet_received)
        self.srv = self.create_service(GetSpectrum, 'get_spectrum', self.get_spectrum_callback)
        thread = threading.Thread(target=self.serial_port_receive, args=(self.ser, self.nsp32))
        thread.daemon = True
        thread.start()

    def get_spectrum_callback(self, request, response):
        self.get_logger().info('Acquiring spectrum...')
        self.nsp32.GetSensorId(0)
        self.nsp32.GetWavelength(0)
        self.nsp32.AcqSpectrum(0, 32, 3, False)

        time.sleep(0.5)
        response.wavelengths = self.wavelengths
        response.spectrum = self.spectrum
        return response

    def data_channel_send_data(self, data):
        if self.ser.isOpen():
            self.ser.write(data)

    def serial_port_receive(self, ser, nsp32):
        while ser.isOpen():
            if ser.in_waiting > 0:
                nsp32.OnReturnBytesReceived(ser.read(ser.in_waiting))

    def on_return_packet_received(self, pkt):
        if pkt.CmdCode == CmdCodeEnum.GetSensorId:
            self.get_logger().info(f'Sensor id = {pkt.ExtractSensorIdStr()}')
        elif pkt.CmdCode == CmdCodeEnum.GetWavelength:
            infoW = pkt.ExtractWavelengthInfo()
            self.wavelengths = list(infoW.Wavelength) 
            self.get_logger().info(f'Wavelengths received.')
            # self.get_logger().info(f'Wavelengths received. Type: {(infoW.Wavelength)}')
        elif pkt.CmdCode == CmdCodeEnum.GetSpectrum:
            infoS = pkt.ExtractSpectrumInfo()
            self.spectrum = list(infoS.Spectrum) 
            self.get_logger().info(f'Spectrum received.')

def main(args=None):
    rclpy.init(args=args)
    spectrum_service = SpectrumService()
    rclpy.spin(spectrum_service)
    spectrum_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()