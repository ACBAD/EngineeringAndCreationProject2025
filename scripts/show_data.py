import matplotlib.pyplot as plt
import json

# 示例数据（替换为你的实际数据）
with open("wheel_ticks.json", "r") as f:
    data = json.load(f)
list_a = data['l_data']
list_b = data['r_data']

if len(list_a) != len(list_b):
    smaller_list = list_a if len(list_a) < len(list_b) else list_b
    while len(list_a) != len(list_b):
        smaller_list.append(smaller_list[-1])

# 生成下标作为x轴数据
x_values = range(len(list_a))

# 创建折线图
plt.plot(x_values, list_a, label='l') # 绘制第一条线
plt.plot(x_values, list_b, label='r')  # 绘制第二条线

# 添加图表元素
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Comparison of Two Lists')
plt.grid(True)
plt.legend()

# 显示图表
plt.show()

