# -*- coding: utf-8 -*-
"""
    lantz.drivers.newport.motionsmc100
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    Implements the drivers to control SMC100 controller

    :copyright: 2018, see AUTHORS for more details.
    :license: GPL, see LICENSE for more details.

    Source: Instruction Manual (Newport)

"""


from lantz.feat import Feat
from lantz.action import Action
from pyvisa import constants
from lantz import Q_, ureg
from lantz.processors import convert_to
if __name__ == '__main__':
    from motion import MotionController, MotionAxis
else:
    from .motion import MotionController, MotionAxis
import time
import numpy as np

ERRORS = {"@": "",
        "A": "Unknown message code or floating point controller address.",
        "B": "Controller address not correct.",
        "C": "Parameter missing or out of range.",
        "D": "Execution not allowed.",
        "E": "home sequence already started.",
        "I": "Execution not allowed in CONFIGURATION state.",
        "J": "Execution not allowed in DISABLE state.",
        "K": "Execution not allowed in READY state.",
        "L": "Execution not allowed in HOMING state.",
        "M": "Execution not allowed in MOVING state.",}


class SMC100(MotionController):
    """ Newport SMC100 motion controller. It assumes all axes to have units mm


    Example:
    import numpy as np
    import lantz
    import visa
    import lantz.drivers.pi.piezo as pi
    from lantz.drivers.newport_motion import SMC100
    from pyvisa import constants
    rm = visa.ResourceManager('@py')
    lantz.messagebased._resource_manager = rm
    ureg = lantz.ureg
    try:
        lantzlog
    except NameError:
        lantzlog = lantz.log.log_to_screen(level=lantz.log.DEBUG)
        lantz.log.log_to_socket(level=lantz.log.DEBUG)
        
    import time
    import numpy as np
    import warnings
    #warnings.filterwarnings(action='ignore')
    print(lantz.messagebased._resource_manager.list_resources())
    stage = SMC100('ASRL/dev/ttyUSB0::INSTR')
    stage.initialize()
    axis0 = stage.axes[0]
    print('Axis id:' + axis0.idn)
    print('Axis position: {}'.format(axis0.position))
    axis0.keypad_disable()
    axis0.position += 0.1 * ureg.mm
    print('Errors: {}'.format(axis0.get_errors()))
    stage.finalize()
    """

    DEFAULTS = {
                'COMMON': {'write_termination': '\r\n',
                           'read_termination': '\r\n', },
                'ASRL': {
                    'timeout': 100,  # ms
                    'encoding': 'ascii',
                    'data_bits': 8,
                    'baud_rate': 57600,
                    'parity': constants.Parity.none,
                    'stop_bits': constants.StopBits.one,
                    #'flow_control': constants.VI_ASRL_FLOW_NONE,
                    'flow_control': constants.VI_ASRL_FLOW_XON_XOFF,  # constants.VI_ASRL_FLOW_NONE,
                    },
                }

    def __init__(self, *args, **kwargs):
        self.motionaxis_class = kwargs.pop('motionaxis_class', MotionAxisSMC100)
        super().__init__(*args, **kwargs)

    def initialize(self):
        super().initialize()
        self.detect_axis()

    @Action()
    def detect_axis(self):
        """ Find the number of axis available.
        
        The detection stops as soon as an empty controller is found.
        """
        self.axes = []
        i = 0
        scan_axes = True
        while scan_axes:
            i += 1
            id = self.query('%dID?' % i)
            if id == '':
                scan_axes = False
            else:
                axis = self.motionaxis_class(self, i, id)
                self.axes.append(axis)
            

            

    
class MotionAxisSMC100(MotionAxis):
    def query(self, command, *, send_args=(None, None), recv_args=(None, None)):
        respons = super().query(command,send_args=send_args, recv_args=recv_args)
        #check for command:
        if not respons[:3] == '{:d}{}'.format(self.num, command[:2]):
            self.log_error('Axis {}: Expected to return command {} instead of {}'.format(self.num, command[:3],respons[:3]))
        return respons[3:]

    def write(self, command, *args, **kwargs):
        super().write(command, *args, **kwargs)
        return self.get_errors()

    @Action()
    def on(self):
        """Put axis on"""
        pass

    @Action()
    def off(self):
        """Put axis on"""
        pass
    
    @Action()
    def get_errors(self):
        ret = self.query('TE?')
        err = ERRORS.get(ret, 'Error {}. Lookup in manual: https://www.newport.com/medias/sys_master/images/images/h11/he1/9117182525470/SMC100CC-SMC100PP-User-s-Manual.pdf'.format(ret))
        if err:
            self.log_error('Axis {} error: {}'.format(self.num, err))
        return err

    @Feat(values={True: '1', False: '0'})
    def is_on(self):
        """
        :return: True is axis on, else false
        """
        return '1'
        
    @Feat()
    def motion_done(self):
        return self.check_position(self.last_set_position)

    @Action()
    def keypad_disable(self):
        return self.write('JD')


if __name__ == '__main__':
    import argparse
    import lantz.log

    parser = argparse.ArgumentParser(description='Test SMC100 driver')
    parser.add_argument('-p', '--port', type=str, default='1',
                        help='Serial port to connect to')

    args = parser.parse_args()
    lantzlog = lantz.log.log_to_screen(level=lantz.log.INFO)
    lantz.log.log_to_socket(lantz.log.DEBUG)

    import lantz
    import visa
    import lantz.drivers.newport_motion
    sm = lantz.drivers.newport_motion.SMC100
    rm = visa.ResourceManager('@py')
    lantz.messagebased._resource_manager = rm

    print(lantz.messagebased._resource_manager.list_resources())

    with sm(args.port) as inst:
    #with sm.via_serial(port=args.port) as inst:
        inst.idn
        # inst.initialize() # Initialize the communication with the power meter
        # Find the status of all axes:
        #for axis in inst.axes:
        #    print('Axis {} Position {} is_on {} max_velocity {} velocity {}'.format(axis.num, axis.position,
        #                                                                            axis.is_on, axis.max_velocity,
        #                                                                            axis.velocity))
