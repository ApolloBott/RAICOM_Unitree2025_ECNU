import time

class Timer:
    def __init__(self):
        self.timer_is_started = False    # 计时器启动标志
        self.time_is_up = False         # 超时标志
        self._start_time = 0.0          # 计时开始时间点
        self._time_threshold = 0.0      # 超时阈值
        self._paused_time = 0.0         # 暂停累计时间
        self._pause_start = 0.0         # 暂停开始时间
        self._is_paused = False         # 暂停标志

    def start_timer(self):
        """ 启动计时器（重置所有状态） """
        self._start_time = time.perf_counter()
        self.timer_is_started = True
        self.time_is_up = False
        self._paused_time = 0.0
        self._is_paused = False

    def get_elapsed(self, time_threshold: float) -> float:
        """
        获取已过去的时间并检查是否超时
        :param time_threshold: 超时判断阈值（单位：秒）
        :return: 实际经过的时间（单位：秒）
        """
        if not self.timer_is_started:
            return 0.0
        
        # 如果当前暂停中，返回暂停前的时间
        if self._is_paused:
            elapsed = self._pause_start - self._start_time - self._paused_time
        else:
            # 计算已过去的时间（减去暂停累计时间）
            elapsed = time.perf_counter() - self._start_time - self._paused_time
        
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

    def pause_timer(self):
        """ 暂停计时器 """
        if self.timer_is_started and not self._is_paused:
            self._pause_start = time.perf_counter()
            self._is_paused = True

    def resume_timer(self):
        """ 恢复计时器 """
        if self.timer_is_started and self._is_paused:
            self._paused_time += time.perf_counter() - self._pause_start
            self._is_paused = False

    @property
    def is_paused(self) -> bool:
        """ 计时器是否暂停（只读属性） """
        return self._is_paused
    
    def print_timer_status(self, context=""):
        """ 打印计时器当前状态，用于调试 """
        if not self.timer_is_started:
            print(f"[TIMER-DEBUG] {context} Timer not started")
            return
        
        current_time = time.perf_counter()
        raw_elapsed = current_time - self._start_time
        
        if self._is_paused:
            effective_elapsed = self._pause_start - self._start_time - self._paused_time
            print(f"[TIMER-DEBUG] {context} PAUSED - Raw: {raw_elapsed:.3f}s, Effective: {effective_elapsed:.3f}s, Paused total: {self._paused_time:.3f}s")
        else:
            effective_elapsed = raw_elapsed - self._paused_time
            print(f"[TIMER-DEBUG] {context} RUNNING - Raw: {raw_elapsed:.3f}s, Effective: {effective_elapsed:.3f}s, Paused total: {self._paused_time:.3f}s")

timer = Timer()