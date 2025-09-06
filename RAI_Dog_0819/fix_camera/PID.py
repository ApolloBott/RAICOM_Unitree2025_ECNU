class PID:
    def __init__(self, kp: float, ki: float, kd: float, set_point: float):
        """
        PID 控制器初始化
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        :param set_point: 设定值
        """
        # 基础参数
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        
        # 运行状态
        self.integral = 0.0        # 积分项累积值
        self.last_input = 0.0      # 上一次输入值
        self.auto_mode = False     # 控制模式标志
        
        # 输出限制（初始值会被后续设置覆盖）
        self.output_min = 0.0
        self.output_max = 255.0
        self.previousError = 0

        # 初始化配置
        self.set_mode(True)               # 启用自动模式
        self.set_output_limits(-1.5, 1.5) # 设置输出限幅

    def set_tunings(self, kp: float, ki: float, kd: float):
        """ 动态调整 PID 参数 """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_output_limits(self, min_val: float, max_val: float):
        """ 设置输出值限幅范围 """
        if min_val >= max_val:
            return  # 无效输入保护
        self.output_min = min_val
        self.output_max = max_val

    def set_mode(self, auto_mode: bool):
        """ 切换控制模式（自动/手动） """
        self.auto_mode = auto_mode
        if not self.auto_mode:
            # 手动模式时重置控制器状态
            self.integral = 0.0
            self.previousError = 0.0

    def compute(self, input_val: float) -> float:
        """ 执行单次 PID 计算 """
        if not self.auto_mode:
            return 0.0  # 手动模式直接返回0

        # 误差计算
        error = self.set_point - input_val
        
        # 积分项计算（带限幅）
        self.integral += self.ki * error
        self.integral = max(min(self.integral, self.output_max), self.output_min)
        
        # 微分项计算（基于输入变化率）
        d_input = input_val - self.last_input
        
        # 输出计算（带限幅）
        output = (
            self.kp * error +
            self.integral -
            self.kd * d_input
        )
        output = max(min(output, self.output_max), self.output_min)
        
        # 保存当前状态
        self.last_input = input_val
        
        return output