# can_config.py
import os
import sys
import can

def setup_canalyst_environment():
    """设置CANalyst-II运行环境"""
    
    # 1. 检查当前工作目录
    print(f"当前工作目录: {os.getcwd()}")
    
    # 2. 检查系统PATH是否包含CANalyst-II驱动路径
    system_path = os.environ.get('PATH', '')
    print("系统PATH包含:")
    for path in system_path.split(';'):
        if 'canalyst' in path.lower() or 'controlcan' in path.lower():
            print(f"  - {path}")
    
    # 3. 检查是否有控制卡DLL文件
    dll_files = ['ControlCAN.dll', 'controlcan.dll', 'zlgcan.dll']
    found_dll = False
    
    for dll in dll_files:
        # 检查当前目录
        if os.path.exists(dll):
            print(f"✓ 在当前目录找到DLL: {dll}")
            found_dll = True
        # 检查System32目录
        system32_path = os.path.join(os.environ.get('SystemRoot', 'C:\\Windows'), 'System32', dll)
        if os.path.exists(system32_path):
            print(f"✓ 在System32目录找到DLL: {dll}")
            found_dll = True
    
    if not found_dll:
        print("✗ 未找到CANalyst-II的DLL文件")
    
    # 4. 打印python-can版本和可用接口
    print(f"\npython-can版本: {can.__version__}")
    print("可用的接口:")
    try:
        for interface in can.interfaces:
            print(f"  - {interface}")
    except:
        print("  无法获取接口列表")
    
    # 5. 检查canalystii接口是否可用
    try:
        import can.interfaces.canalystii
        print("✓ canalystii接口模块可用")
    except ImportError as e:
        print(f"✗ canalystii接口模块不可用: {e}")
    
    return found_dll

if __name__ == "__main__":
    setup_canalyst_environment()