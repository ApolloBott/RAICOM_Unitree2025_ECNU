import time

class Timer:
    def __init__(self):
        self.timer_is_started = False    # 计时器启动标志
        self.time_is_up = False         # 超时标志
        self._start_time = 0.0          # 计时开始时间点
        self._time_threshold = 0.0      # 超时阈值

    def start_timer(self):
        """ 启动计时器（重置所有状态） """
        self._start_time = time.perf_counter()
        self.timer_is_started = True
        self.time_is_up = False

    def get_elapsed(self, time_threshold: float) -> float:
        """
        获取已过去的时间并检查是否超时
        :param time_threshold: 超时判断阈值（单位：秒）
        :return: 实际经过的时间（单位：秒）
        """
        if not self.timer_is_started:
            return 0.0
        
        # 计算已过去的时间
        elapsed = time.perf_counter() - self._start_time
        
        # 超时判断与状态更新
        if elapsed >= time_threshold:
            self.time_is_up = True
            self.timer_is_started = False
        
        return elapsed

    @property
    def is_time_up(self) -> bool:
        """ 是否已超时（只读属性） """
        return self.time_is_up

    @property
    def is_timer_started(self) -> bool:
        """ 计时器是否在运行（只读属性） """
        return self.timer_is_started
timer = Timer()