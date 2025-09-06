from Run import Run
from StateMachine import Transition
from StateMachine import States
from StateMachine import Events
from StateMachine import StateMachine
#from Actions import actions
from Actions import Actions
from Img_Processor import Img_Processor
#from Sender import sender_pi
#from Receiver import receiver_13
#from Receiver import receiver_15
from Sender import Sender
from Receiver import Receiver
import threading
import time
HOST_PC = "192.168.123.15"
HOST_PI = "192.168.123.161"
PORT_TO_PI =  52013
PORT_FROM_13 =  52014
PORT_FROM_15 =  12345



def init_sm(sm: StateMachine, actions: Actions):
    '''
    sm.add_trs(States.S_IDLE, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    sm.add_trs(States.S_IDLE, Events.E_START, States.S_IDLE, actions.f_go_start)
    sm.add_trs(States.S_GO_S, Events.E_TURN_L, States.S_TURN_L, actions.f_turn_left)
    sm.add_trs(States.S_GO_S, Events.E_TURN_R, States.S_TURN_R, actions.f_turn_right)
    sm.add_trs(States.S_TURN_L, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    sm.add_trs(States.S_TURN_R, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    #straight
    sm.add_trs(States.S_GO_S, Events.E_ENT_CIR, States.S_ENT_CIR, actions.f_enter_circle)
    sm.add_trs(States.S_GO_S, Events.E_IGN_CIR, States.S_IGN_CIR, actions.f_ignore_circle)
    sm.add_trs(States.S_GO_S, Events.E_ENT_CRO, States.S_ENT_CRO, actions.f_enter_crossing)
    sm.add_trs(States.S_GO_S, Events.E_IGN_CRO, States.S_IGN_CRO, actions.f_ignore_crossing)
    #circle
    sm.add_trs(States.S_ENT_CIR, Events.E_IN_CIR, States.S_ENT_CIR ,actions.f_in_circle)
    sm.add_trs(States.S_IN_CIR, Events.E_EXT_CIR, States.S_EXT_CIR, actions.f_exit_circle)
    sm.add_trs(States.S_EXT_CIR, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    sm.add_trs(States.S_IGN_CIR, Events.E_GO_S, States.S_GO_S, actions.f_gp_straight)
    #crossing
    sm.add_trs(States.S_ENT_CRO, Events.E_IN_CRO, States.S_IN_CRO, actions.f_in_crossing)
    sm.add_trs(States.S_IN_CRO, Events.E_EXT_CRO, States.S_EXT_CRO, actions.f_exit_crossing)
    sm.add_trs(States.S_EXT_CRO, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    sm.add_trs(States.S_IGN_CRO, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)
    '''
    sm.add_trs(States.S_GO_S, Events.E_GO_S, States.S_GO_S, actions.f_go_straight)


    #sm.init(States.S_IDLE,States.S_IDLE)

if __name__ == "__main__":

    
    #print("Now Waiting for Frames")
    # 创建并启动线程
    #transition = Transition()
    actions = Actions()
    sm = StateMachine()
    run = Run()
    receiver_15 = Receiver(0, "192.168.123.15", 12345)
    sender_pi = Sender("192.168.123.161", 52013)
    img_processor = Img_Processor()
    init_sm(sm,actions)
    sender_pi.start()
    try:
        run.run(img_processor,sm,receiver_15,sender_pi,actions)  # 假设recver_15已定义
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    '''
    send_th = threading.Thread(
        target=sender_pi.send_mes_th,
        daemon=True  # 守护线程随主线程退出
    )
    
    head_th = threading.Thread(
            target=img_processor.recog_hindrance,
            args=(receiver_13,),  # 假设recver_13已定义
            daemon=True
        )
    '''
    #send_th.start()
    #head_th.start()
    








