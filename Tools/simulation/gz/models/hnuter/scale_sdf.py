import re
import os

def scale_sdf_mass_and_inertia(input_file, output_file, target_mass):
    if not os.path.exists(input_file):
        print(f"找不到文件: {input_file}")
        return

    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. 提取所有 <mass> 的值并计算当前总质量
    masses = [float(m) for m in re.findall(r'<mass>\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*</mass>', content)]
    current_total_mass = sum(masses)

    if current_total_mass == 0:
        print("解析错误：总质量为 0。")
        return

    scale_ratio = target_mass / current_total_mass
    
    print(f"📊 当前 SDF 总质量: {current_total_mass:.4f} kg")
    print(f"🎯 目标总质量: {target_mass:.4f} kg")
    print(f"⚙️ 全局缩放系数: {scale_ratio:.6f}\n")

    # 2. 替换函数：将匹配到的数值乘以缩放系数
    def scale_match(match):
        tag = match.group(1)
        val = float(match.group(2))
        new_val = val * scale_ratio
        return f"<{tag}>{new_val:.8g}</{tag}>"

    # 需要缩放的标签：质量 + 所有惯性张量分量
    tags_to_scale = ['mass', 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
    pattern = r'<(' + '|'.join(tags_to_scale) + r')>\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*</\1>'

    # 3. 执行全局替换
    new_content = re.sub(pattern, scale_match, content)

    # 4. 保存新文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"✅ 成功！缩放后的模型已保存为: {output_file}")
    
    # 5. 打印控制器需要同步修改的参数
    print("\n⚠️ 请务必在你的 hnuter_external_controller.py 中更新以下参数：")
    print(f"self.mass = {target_mass}  # kg")
    print(f"self.J = np.diag([{0.15 * scale_ratio:.6f}, {0.15 * scale_ratio:.6f}, {0.25 * scale_ratio:.6f}])")

if __name__ == '__main__':
    # 假设你的原文件叫 model.sdf，新文件叫 model_light.sdf，目标质量 4.5 kg
    scale_sdf_mass_and_inertia('model.sdf', 'model_light.sdf', 4.5)