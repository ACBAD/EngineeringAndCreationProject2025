#!/usr/bin/env python3
import select
import os
import time
import sys

# 配置需要监控的GPIO列表
GPIO_LIST = [460, 461, 462, 463]  # 修改为你实际的GPIO编号
CALLBACK_SCRIPT = "/home/khadas/race.sh"


def check_root_permission():
    """检查是否以root权限运行"""
    if os.geteuid() != 0:
        print("\033[91m错误：此程序必须使用sudo运行\033[0m")
        print("请使用以下命令运行：")
        print(f"  sudo {sys.argv[0]}")
        sys.exit(1)


class GPIOListener:
    def __init__(self, gpio_list):
        self.gpio_info = {}
        self.poll = select.poll()

        # 初始化所有GPIO
        for gpio in gpio_list:
            self._init_gpio(gpio)
            self._setup_gpio(gpio)

    @staticmethod
    def _init_gpio(gpio):
        """GPIO初始化：导出和方向设置"""
        value_path = f"/sys/class/gpio/gpio{gpio}/value"
        dir_path = f"/sys/class/gpio/gpio{gpio}/direction"

        # 导出GPIO
        if not os.path.exists(value_path):
            try:
                with open("/sys/class/gpio/export", "w") as f:
                    f.write(str(gpio))
                time.sleep(0.1)  # 等待系统创建文件
            except Exception as e:
                raise RuntimeError(f"导出GPIO {gpio} 失败: {str(e)}")

        # 强制设置为输入模式
        try:
            with open(dir_path, "w") as f:
                f.write("in")
        except IOError as e:
            raise RuntimeError(f"设置GPIO {gpio} 方向失败: {str(e)}")

    def _setup_gpio(self, gpio):
        """配置GPIO监听参数"""
        edge_path = f"/sys/class/gpio/gpio{gpio}/edge"
        value_path = f"/sys/class/gpio/gpio{gpio}/value"

        # 设置仅响应下降沿
        try:
            with open(edge_path, "w") as f:
                f.write("falling")
        except IOError as e:
            raise RuntimeError(f"设置GPIO {gpio} 中断模式失败: {str(e)}")

        # 打开value文件
        try:
            fd = os.open(value_path, os.O_RDONLY)
            os.read(fd, 1)  # 清除初始状态
        except OSError as e:
            raise RuntimeError(f"打开GPIO {gpio} 值文件失败: {str(e)}")

        # 注册到poll
        self.poll.register(fd, select.POLLPRI)
        self.gpio_info[fd] = {
            "gpio": gpio,
            "path": value_path
        }

    @staticmethod
    def _read_gpio_value(fd):
        os.lseek(fd, 0, os.SEEK_SET)
        return os.read(fd, 1).decode().strip()

    def run(self):
        try:
            print(f"\033[92mGPIO监控服务已启动（下降沿触发模式），监听: {GPIO_LIST}\033[0m")
            while True:
                events = self.poll.poll()
                for fd, event in events:
                    if event & select.POLLPRI:
                        current_value = self._read_gpio_value(fd)
                        gpio_num = self.gpio_info[fd]["gpio"]
                        if current_value == 0:
                            continue
                        if gpio_num == 461:
                            print(f'{gpio_num}被触发')
                            os.system(f"{CALLBACK_SCRIPT} {gpio_num}")
                            time.sleep(1)
        except KeyboardInterrupt:
            print("\n\033[93m接收到中断信号，停止监控...\033[0m")
        finally:
            # 清理资源
            for fd in self.gpio_info:
                os.close(fd)


if __name__ == "__main__":
    check_root_permission()
    try:
        listener = GPIOListener(GPIO_LIST)
        listener.run()
    except Exception as ge:
        print(f"\033[91m致命错误: {str(ge)}\033[0m", file=sys.stderr)
        sys.exit(1)
