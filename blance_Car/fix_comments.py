# -*- coding: utf-8 -*-
import re

# 读取文件 (尝试不同编码)
try:
    with open(r'h:\keil\project\blance_Car\Core\Src\main.c', 'r', encoding='gb2312') as f:
        content = f.read()
except:
    with open(r'h:\keil\project\blance_Car\Core\Src\main.c', 'r', encoding='gbk') as f:
        content = f.read()

# 定义替换规则 (乱码 -> 正确注释)
replacements = [
    # 超声波相关
    (r'// \?{10,}浽\?{10,}', '// 读取超声波距离并保存到全局变量'),
    (r'// \?{10,} send_buf', '// 格式化数据到发送缓冲区'),
    (r'// \?{10,} send_buf \?{5,}', '// 发送超声波数据'),
    (r'// \?{10,} Txbuff', '// 发送其他数据'),
    
    # 蓝牙接收相关
    (r'// \?{10,}', '// 处理接收数据'),
    (r'Blu_rx_ready = 0; // \?{5,}', 'Blu_rx_ready = 0; // 清除标志位'),
    
    # 指令解析相关
    (r'// \?{5,}\'\]\'\?{10,}', '// 找到最后一个闭合括号 ]'),
    (r'// \?{5,}\'\[\'\?{10,}', '// 找到完整的指令包'),
    (r'p = start_ptr; // \?{10,}', 'p = start_ptr; // 指向完整指令'),
    (r'// \?{5,}\(int \?{5,}\)', '// 定义临时变量 (int 类型)'),
    (r'// \?{10,}$', '// 如果收到有效指令'),
    
    # PID相关
    (r'/\* -+ A\. \?{5,} \(Upright\) -+ \*/', '/* ----------- A. 直立环 (Upright) ----------- */'),
    (r'/\* -+ B\. \?{5,} \(Track\) -+ \*/', '/* ----------- B. 循迹环 (Track) ----------- */'),
    (r'/\* -+ C\. \?{5,} \(Speed\) -+ \*/', '/* ----------- C. 速度环 (Speed) ----------- */'),
    
    # 其他
    (r'//\?{5,}0\?{5,}', '// 如果速度为0，强制停车'),
    (r'Car_dir = \'S\';  // \?{10,}', "Car_dir = 'S';  // 停止状态"),
    (r'// \?{5,}DMA\?{5,}', '// 重新开启DMA接收'),
    (r'// \?{5,}\?{5,}\?{5,}', '// 看门狗超时停车'),
    (r'Car_speed = 0; // \?{10,}', 'Car_speed = 0; // 速度清零'),
    (r"Car_dir = 'S'; // \?{5,}", "Car_dir = 'S'; // 方向复位"),
    
    # 机械零点
    (r'//\?{5,}\?{5,}', '// 机械零点'),
    
    # 调试模式
    (r'//\?{5,}\?{5,}\?{5,}', '// 调试模式开关'),
]

# 执行替换
for pattern, replacement in replacements:
    content = re.sub(pattern, replacement, content, flags=re.MULTILINE)

# 保存文件
with open(r'h:\keil\project\blance_Car\Core\Src\main.c', 'w', encoding='gb2312') as f:
    f.write(content)

print("Done!")
