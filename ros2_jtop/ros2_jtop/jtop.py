from jtop import jtop
import os
from jtop_orinnx_msgs.msg import (
    Jtop, 
    Cpu, CpuFrequency, IdleState, 
    Gpu, GpuFrequency, GpuStatus,
    Mem, Emc, Ram, Swap, Zram,
    TempList, Temp)
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class ROS2Jtop(Node):
    def __init__(self):
        super().__init__('jtop')
        self.jtop_pub_ = self.create_publisher(Jtop, 'jtop', 1)

        self.jtop_msg_ = Jtop()

        interval_config = ParameterDescriptor()
        interval_config.description = "jtop frequency in seconds"
        self.declare_parameter('interval', 0.5, interval_config)

        num_cpus_config = ParameterDescriptor()
        num_cpus_config.description = "the number pf cpu cores (e.g. 8 cores for Jetson Orin NX 16GB)"
        self.declare_parameter('max_num_cpus', 8, num_cpus_config)

        interval = self.get_parameter('interval')._value
        self.timer = self.create_timer(interval, self.jtop_callback)
        self.num_cpus_ = self.get_parameter('max_num_cpus')._value
        self.get_logger().info('interval: {}'.format(interval))
        self.get_logger().info('num_cpus: {}'.format(self.num_cpus_))

        self.jtop_ = jtop(interval=0.5)
        self.jtop_.start()

    def jtop_callback(self):
        if self.jtop_.ok():
            self.jtop_msg_ = Jtop()
            self.jtop_msg_.header.stamp = self.get_clock().now().to_msg()
            
            self.get_cpu_info()
            self.get_gpu_info()
            self.get_memory_info()
            self.get_temperature_info()
            self.get_power_mode()

            self.jtop_pub_.publish(self.jtop_msg_)

    def get_cpu_info(self):
        """
         'cpu': [
            {'online': True, 'governor': 'schedutil', 'freq': {'min': 729600, 'max': 1190400, 'cur': 1190400}, 'info_freq': {'min': 115200, 'max': 1984000, 'cur': 1190400}, 'idle_state': {'WFI': 0, 'c7': 0}, 'model': '', 'user': 11.76470588235294, 'nice': 0.0, 'system': 3.9215686274509802, 'idle': 83.33333333333334}, 
            {'online': True, 'governor': 'schedutil', 'freq': {'min': 729600, 'max': 1190400, 'cur': 1190400}, 'info_freq': {'min': 115200, 'max': 1984000, 'cur': 1190400}, 'idle_state': {'WFI': 0, 'c7': 0}, 'model': '', 'user': 6.122448979591836, 'nice': 0.0, 'system': 3.061224489795918, 'idle': 90.81632653061224}, 
            {'online': True, 'governor': 'schedutil', 'freq': {'min': 729600, 'max': 1190400, 'cur': 1190400}, 'info_freq': {'min': 115200, 'max': 1984000, 'cur': 1190400}, 'idle_state': {'WFI': 0, 'c7': 0}, 'model': '', 'user': 2.0202020202020203, 'nice': 0.0, 'system': 3.0303030303030303, 'idle': 93.93939393939394}, 
            {'online': True, 'governor': 'schedutil', 'freq': {'min': 729600, 'max': 1190400, 'cur': 1190400}, 'info_freq': {'min': 115200, 'max': 1984000, 'cur': 1190400}, 'idle_state': {'WFI': 0, 'c7': 0}, 'model': '', 'user': 17.647058823529413, 'nice': 0.0, 'system': 4.901960784313726, 'idle': 76.47058823529412}, 
            {'online': False, 'model': ''}, 
            {'online': False, 'model': ''}, 
            {'online': False, 'model': ''}, 
            {'online': False, 'model': ''}
        ]
        """

        cpu_info = self.jtop_.cpu['cpu']
        for id in range(self.num_cpus_):
            cpu = Cpu()
            cpu.online = cpu_info[id]['online']

            if cpu.online :
                cpu.governor = cpu_info[id]['governor']

                cpu.freq = CpuFrequency()
                cpu.freq.min = cpu_info[id]['freq']['min']
                cpu.freq.cur = cpu_info[id]['freq']['cur']
                cpu.freq.max = cpu_info[id]['freq']['max']
                
                cpu.info_freq = CpuFrequency()
                cpu.info_freq.min = cpu_info[id]['info_freq']['min']
                cpu.info_freq.cur = cpu_info[id]['info_freq']['cur']
                cpu.info_freq.max = cpu_info[id]['info_freq']['max']

                cpu.idle_state = IdleState()
                cpu.idle_state.wfi = cpu_info[id]['idle_state']['WFI']
                cpu.idle_state.c7 = cpu_info[id]['idle_state']['c7']

                cpu.user = cpu_info[id]['user']
                cpu.nice = cpu_info[id]['nice']
                cpu.system = cpu_info[id]['system']
                cpu.idle = cpu_info[id]['idle']
            cpu.model = cpu_info[id]['model']
            self.jtop_msg_.cpus.append(cpu)

    def get_gpu_info(self):
        """
        'gpu': {'type': 'integrated', 'status': {'railgate': False, 'tpc_pg_mask': False, '3d_scaling': True, 'load': 0.0}, 'freq': {'governor': 'nvhost_podgov', 'cur': 306000, 'max': 612000, 'min': 306000, 'GPC': [305956]}, 'power_control': 'auto'}
        """

        gpu_info = self.jtop_.gpu['gpu']
        gpu = Gpu()
        
        gpu.type = gpu_info['type']

        gpu.status = GpuStatus()
        gpu.status.railgate = gpu_info['status']['railgate']
        gpu.status.tpc_pg_mask = gpu_info['status']['tpc_pg_mask']
        gpu.status.scaling_3d = gpu_info['status']['3d_scaling']
        gpu.status.load = gpu_info['status']['load']

        gpu.freq = GpuFrequency()
        gpu.freq.governor = gpu_info['freq']['governor']
        gpu.freq.cur = gpu_info['freq']['cur']
        gpu.freq.max = gpu_info['freq']['max']
        gpu.freq.min = gpu_info['freq']['min']
        for i in range(len(gpu_info['freq']['GPC'])):
            gpu.freq.gpc.append(gpu_info['freq']['GPC'][i])  # Assuming GPC is a list and we want the first value

        gpu.power_control = gpu_info['power_control']

        self.jtop_msg_.gpu = gpu

    def get_memory_info(self):
        """
        'RAM': {'tot': 16032400, 'used': 1246060, 'free': 13954968, 'buffers': 78820, 'cached': 834948, 'shared': 22948, 'lfb': 834}, 
        'SWAP': {
            'tot': 8016192, 'used': 0, 'cached': 0, 'table': 
            {
            '/dev/zram0': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram1': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram2': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram3': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram4': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram5': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram6': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}, 
            '/dev/zram7': {'type': 'zram', 'prio': 5, 'size': 1002024, 'used': 0, 'boot': False}
            }   
        }, 
        'EMC': {'cur': 2133000, 'max': 3199000, 'min': 204000, 'override': 0, 'val': 0, 'online': True}
        """
        
        ram_info = self.jtop_.memory['RAM']
        ram = Ram()
        ram.total = ram_info['tot']
        ram.used = ram_info['used']
        ram.free = ram_info['free']
        ram.buffers = ram_info['buffers']
        ram.cached = ram_info['cached']
        ram.shared = ram_info['shared']
        ram.lfb = ram_info['lfb']
        self.jtop_msg_.mem.ram = ram

        swap_info = self.jtop_.memory['SWAP']
        swap = Swap()
        swap.tot = swap_info['tot']
        swap.used = swap_info['used']
        swap.cached = swap_info['cached']

        for device_name, device_info in swap_info['table'].items():
            swap_device = Zram()
            swap_device.type = device_info['type']
            swap_device.prio = device_info['prio']
            swap_device.size = device_info['size']
            swap_device.used = device_info['used']
            swap_device.boot = device_info['boot']
            swap.table.append(swap_device)
        
        self.jtop_msg_.mem.swap = swap

        emc_info = self.jtop_.memory['EMC']
        emc = Emc()
        emc.cur = emc_info['cur']
        emc.max = emc_info['max']
        emc.min = emc_info['min']
        emc.override = emc_info['override']
        emc.val = emc_info['val']
        emc.online = emc_info['online']
        self.jtop_msg_.mem.emc = emc

    def get_temperature_info(self):
        """
        'cpu': {'temp': 53.187, 'online': True}, 
        'cv0': {'temp': 51.0, 'online': True}, 
        'cv1': {'temp': 51.062, 'online': True}, 
        'cv2': {'temp': 51.218, 'online': True}, 
        'gpu': {'temp': 50.843, 'online': True}, 
        'soc0': {'temp': 53.0, 'online': True}, 
        'soc1': {'temp': 54.218, 'online': True}, 
        'soc2': {'temp': 50.906, 'online': True}, 
        'tj': {'temp': 54.218, 'online': True}
        """

        temp_info = self.jtop_.temperature
        temps = TempList()
        temps.cpu.temp = temp_info['cpu']['temp']
        temps.cpu.online = temp_info['cpu']['online']

        temps.cv0.temp = temp_info['cv0']['temp']
        temps.cv0.online = temp_info['cv0']['online']

        temps.cv1.temp = temp_info['cv1']['temp']
        temps.cv1.online = temp_info['cv1']['online']

        temps.cv2.temp = temp_info['cv2']['temp']
        temps.cv2.online = temp_info['cv2']['online']

        temps.gpu.temp = temp_info['gpu']['temp']
        temps.gpu.online = temp_info['gpu']['online']

        temps.soc0.temp = temp_info['soc0']['temp']
        temps.soc0.online = temp_info['soc0']['online']

        temps.soc1.temp = temp_info['soc1']['temp']
        temps.soc1.online = temp_info['soc1']['online']

        temps.soc2.temp = temp_info['soc2']['temp']
        temps.soc2.online = temp_info['soc2']['online']

        temps.tj.temp = temp_info['tj']['temp']
        temps.tj.online = temp_info['tj']['online']

        self.jtop_msg_.temps = temps

    def get_power_mode(self):
        self.jtop_msg_.nvpmodel = str(self.jtop_.nvpmodel)

def main(args=None):
    rclpy.init(args=args)
    jtop_node = ROS2Jtop()
    rclpy.spin(jtop_node)
    jtop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # with jtop() as jetson:
    #     while jetson.ok():
    
    #         print(jetson.cpu)
    #         print(jetson.cpu['cpu'][0])
    #         print(jetson.cpu['cpu'][1])
    #         print(jetson.cpu['cpu'][2])
    #         print(jetson.cpu['cpu'][3])
    #         print(jetson.cpu['cpu'][4])
    #         print(jetson.cpu['cpu'][5])
    #         print(jetson.cpu['cpu'][6])
    #         print(jetson.cpu['cpu'][7])

    #         print(jetson.gpu)
    #         print(jetson.gpu['gpu'])
    #         print(jetson.gpu['gpu']['status'])

    #         print(jetson.temperature)

    #         print(jetson.memory)

    #         print(jetson.nvpmodel)
