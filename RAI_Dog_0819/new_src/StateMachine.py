from enum import Enum
from typing import Dict, List, Callable, Optional
import inspect
from Img_Processor import Img_Processor
# 状态枚举定义
class States(Enum):
    S_IDLE = 0
    S_START = 1
    S_GO_S = 2
    S_TURN_R = 3
    S_TURN_L = 4
    S_AVOID = 5
    S_ENT_CIR = 6
    S_IN_CIR = 7
    S_EXT_CIR = 8
    S_IGN_CIR = 9
    S_ENT_CRO = 10
    S_IN_CRO = 11
    S_EXT_CRO = 12
    S_IGN_CRO = 13
    S_FINISH = 14

# 事件枚举定义
class Events(Enum):
    E_KEEP_IDLE = 0
    E_START = 1
    E_GO_S = 2
    E_TURN_R = 3
    E_TURN_L = 4
    E_AVOID = 5
    E_ENT_CIR = 6
    E_IN_CIR = 7
    E_EXT_CIR = 8
    E_IGN_CIR = 9
    E_ENT_CRO = 10
    E_IN_CRO = 11
    E_EXT_CRO = 12
    E_IGN_CRO = 13
    E_FINISH = 14
    # 状态转移数据结构
class Transition:
    def __init__(self, src: States, event: Events, dst: States, action: Callable[[Img_Processor], None]):
        self.src = src
        self.event = event
        self.dst = dst
        self.action = action
class StateMachine:
    def __init__(self):
        # 状态到字符串的映射
        self.enum_str_map: Dict[States, str] = {
            state: f"s_{state.name.split('_', 1)[1].lower()}" 
            for state in States
        }
        #self.init(States.S_IDLE, States.S_IDLE)
        self.init(States.S_GO_S, States.S_GO_S)

    def init(self, current_state: States, last_state: Optional[States] = None):
        self.current_state = current_state
        self.last_state = last_state if last_state else current_state
        self.state_change = False
        self.transitions: List[Transition] = []
       #print(f"StateMachine initialized at {hex(id(self))}")

    def add_trs(self, src_state: States, trigger: Events, dst_state: States, action: Callable):
        self.transitions.append(Transition(src_state, trigger, dst_state, action))

    def take_action(self, event: Events,img_processor: Img_Processor):
        self.state_change = True
        for transition in self.transitions:
            if transition.src == self.current_state and transition.event == event:
                if transition.action and inspect.isfunction(transition.action):
                    transition.action(img_processor)
                self.last_state = self.current_state
                self.current_state = transition.dst
                return
        self.state_change = False

    def repeat_action(self,img_processor: Img_Processor):
        if not self.state_change:
            for transition in self.transitions:
                if transition.src == self.last_state and transition.dst == self.current_state:
                    if transition.action and inspect.isfunction(transition.action):
                        transition.action(img_processor)
                    break
    
    @property
    def state_str(self) -> str:
        return self.enum_str_map.get(self.current_state, "UNKNOWN")

    def print_adr(self):
        print(f"StateMachine instance at {hex(id(self))}")
sm = StateMachine()
    