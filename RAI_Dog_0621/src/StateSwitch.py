from StateMachine import States
from StateMachine import Events
from StateMachine import sm
from Actions import actions
#from Img_Processor import img_processor
from Img_Processor import Img_Processor

class StateSwitch:
    def __init__(self):

        pass

    def state_switch(self,img_processor: Img_Processor):

        if sm.current_state == States.S_IDLE:
            if actions.micore_s == 4:
                actions.micore_s = 0
                sm.take_action(Events.E_GO_S)

        elif sm.current_state == States.S_GO_S:

            pass

        elif sm.current_state == States.S_ENT_CIR:

            if img_processor.left_top[0] < 20 or img_processor.left_rightest[1] > 100:
                actions.micore_s = 0
                sm.take_action(Events.E_IN_CIR)
        
        elif sm.current_state == States.S_IN_CIR:

            #识别黑色路线
            pass

        elif sm.current_state == States.S_EXT_CIR:

            #识别tag码
            pass

        elif sm.current_state == States.S_ENT_CRO:

            if actions.micro_s == 2:
                actions.micro_s == 0
                sm.take_action(Events.E_IN_CRO)

        elif sm.current_state == States.S_EXT_CRO:

            #识别黑色线条
            pass

        elif sm.current_state == States.S_IGN_CRO:

            if (img_processor.width_map[9] + img_processor.width_map[10] + img_processor.width_map[11]) / 3 < 290:
                 actions.micro_s == 0
                 sm.take_action(Events.E_GO_S)
stateswitch = StateSwitch()

        



            



