# -*- coding: utf-8 -*-
"""
    lantz.drivers.newport.motion axis
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    General class that implements the commands used for several newport motion
    drivers

    :copyright: 2018, see AUTHORS for more details.
    :license: GPL, see LICENSE for more details.

    Source: Instruction Manual (Newport)

"""


from lantz.feat import Feat
from lantz.action import Action
from lantz.messagebased import MessageBasedDriver
from pyvisa import constants
from lantz import Q_, ureg
from lantz.processors import convert_to
import time
import numpy as np

#  Add generic units:
# ureg.define('unit = unit')
# ureg.define('encodercount = count')
# ureg.define('motorstep = step')


class MotionControllerBase(MessageBasedDriver):
    """ Newport motion controller. It assumes all axes to have units mm

    """

    DEFAULTS = {
                'COMMON': {'write_termination': '\r\n',
                           'read_termination': '\r\n', },
                'ASRL': {
                    'timeout': 10,  # ms
                    'encoding': 'ascii',
                    'data_bits': 8,
                    'baud_rate': 57600,
                    'parity': constants.Parity.none,
                    'stop_bits': constants.StopBits.one,
                    'flow_control': constants.VI_ASRL_FLOW_RTS_CTS,
                    },
                }


class MotionController(MotionControllerBase):
    def initialize(self):
        super().initialize()

    @Feat()
    def idn(self):
        raise AttributeError('Not implemented')
        # return self.query('ID?')

    @Action()
    def detect_axis(self):
        """ Find the number of axis available.

        The detection stops as soon as an empty controller is found.
        """
        pass

    @Action()
    def get_errors(self):
        raise AttributeError('Not implemented')

    @Feat(read_once=False)
    def position(self):
        return [axis.position for axis in self.axes]

    @Feat(read_once=False)
    def _position_cached(self):
        return [axis.recall('position') for axis in self.axes]

    @position.setter
    def position(self, pos):
        """Move to position (x,y,...)"""
        return self._position(pos)

    @Action()
    def _position(self, pos, read_pos=None, wait_until_done=True):
        """Move to position (x,y,...)"""
        if read_pos is not None:
            self.log_error('kwargs read_pos for function _position is deprecated')

        for p, axis in zip(pos, self.axes):
            if p is not None:
                axis._set_position(p, wait=False)
        if wait_until_done:
            for p, axis in zip(pos, self.axes):
                if p is not None:
                    axis._wait_until_done()
                    axis.check_position(p)
            return self.position

        return pos

    @Action()
    def motion_done(self):
        for axis in self.axes:
            axis._wait_until_done()

    def finalize(self):
        for axis in self.axes:
            if axis is not None:
                del (axis)
        super().finalize()


class MotionAxis(MotionControllerBase):
    def __init__(self, parent, num, id, *args, **kwargs):
        self.parent = parent
        self.num = num
        self._idn = id
        self.wait_time = 0.01  # in seconds * Q_(1, 's')
        self.backlash = 0
        self.wait_until_done = True
        self.accuracy = 0.001  # in units reported by axis
        # Fill position cache:
        self.position
        self.last_set_position = self.position.magnitude

    def __del__(self):
        self.parent = None
        self.num = None

    def query(self, command, *, send_args=(None, None), recv_args=(None, None)):
        return self.parent.query('{:d}{}'.format(self.num, command),
                                 send_args=send_args, recv_args=recv_args)

    def write(self, command, *args, **kwargs):
        return self.parent.write('{:d}{}'.format(self.num, command),
                                 *args, **kwargs)

    @Feat()
    def idn(self):
        return self.query('ID?')

    @Action()
    def on(self):
        """Put axis on"""
        self.write('MO')

    @Action()
    def off(self):
        """Put axis on"""
        self.write('MF')

    @Feat(values={True: '1', False: '0'})
    def is_on(self):
        """
        :return: True is axis on, else false
        """
        return self.query('MO?')

    @Action(units='mm')
    def define_home(self, val=0):
        """Remap current position to home (0), or to new position

        :param val: new position"""
        self.write('DH%f' % val)

    @Action()
    def home(self):
        """Execute the HOME command"""
        self.write('OR')

    @Feat(units='mm')
    def position(self):
        return self.query('TP?')

    @position.setter
    def position(self, pos):
        """
        Waits until movement is done if self.wait_until_done = True.

        :param pos: new position
        """
        # First do move to extra position if necessary
        self._set_position(pos, wait=self.wait_until_done)

    @Action(units=['mm', None, None])
    def _set_position(self, pos, wait=None, force=False):
        """
        Move to an absolute position, taking into account backlash.

        When self.backlash is to a negative value the stage will always move
         from low to high values. If necessary, a extra step with length
         self.backlash is set.

        :param pos: New position in mm
        :param wait: wait until stage is finished
        :param force: Do not check if pos is close to current position
        """

        if not self.is_on:
            self.log_error('Axis not enabled. Not moving!')
            return

        # Check if setting position is really nececcary
        current_value = self.refresh('position')
        if not force and np.isclose(pos, current_value, atol=self.accuracy):
            self.log_info('No need to set {} = {} (current={}, force={}, '
                              'accuracy={})', 
                              'position', pos, current_value, force, 
                              self.accuracy)
            return

        # First do move to extra position if necessary
        if self.backlash:
            position = self.position.magnitude
            backlash = convert_to('mm', on_dimensionless='ignore'
                                   )(self.backlash).magnitude
            if (backlash < 0 and position > pos) or\
               (backlash > 0 and position < pos):

                self.log_info('Using backlash')
                self.__set_position(pos + backlash)
                self._wait_until_done()

        # Than move to final position
        self.__set_position(pos)
        if wait:
            self._wait_until_done()
            self.check_position(pos)

    def __set_position(self, pos):
        """
        Move stage to a certain position
        :param pos: New position
        """
        self.write('PA%f' % (pos))
        self.last_set_position = pos

    @Action(units='mm')
    def check_position(self, pos):
        '''Check is stage is at expected position'''
        if np.isclose(self.position, pos, atol=self.accuracy):
            return True
        self.log_error('Position accuracy {} is not reached.'
                       'Expected: {}, measured: {}'.format(self.accuracy,
                                                           pos,
                                                           self.position))
        return False

    @Feat(units='mm/s')
    def velocity(self):
        return float(self.query('VA?'))

    @velocity.setter
    def velocity(self, velocity):
        """
        :param velocity: Set the velocity that the axis should use when moving
        :return:
        """
        self.write('VA%f' % (velocity))

    @Feat(units='mm/s**2')
    def acceleration(self):
        return float(self.query('VA?'))

    @acceleration.setter
    def acceleration(self, acceleration):
        """
        :param acceleration: Set the acceleration that the axis should use
                             when starting
        :return:
        """
        self.write('AC%f' % (acceleration))

    @Action()
    def stop(self):
        """Emergency stop"""
        self.write('ST')

    @Feat(values={True: '1', False: '0'})
    def motion_done(self):
        return self.query('MD?')

    # Not working yet, see https://github.com/hgrecco/lantz/issues/35
    # @Feat(values={Q_('encodercount'): 0,
    #                     Q_('motor step'): 1,
    #                     Q_('millimeter'): 2,
    #                     Q_('micrometer'): 3,
    #                     Q_('inches'): 4,
    #                     Q_('milli-inches'): 5,
    #                     Q_('micro-inches'): 6,
    #                     Q_('degree'): 7,
    #                     Q_('gradian'): 8,
    #                     Q_('radian'): 9,
    #                     Q_('milliradian'): 10,
    #                     Q_('microradian'): 11})
    #@Feat()
    #def units(self):
    #    ret = int(self.query(u'SN?'))
    #    vals = {0 :'encoder count',
    #            1 :'motor step',
    #            2 :'millimeter',
    #            3 :'micrometer',
    #            4 :'inches',
    #            5 :'milli-inches',
    #            6 :'micro-inches',
    #            7 :'degree',
    #            8 :'gradian',
    #            9 :'radian',
    #            10:'milliradian',
    #            11:'microradian',}
    #    return vals[ret]

    # @units.setter
    # def units(self, val):
    #     self.parent.write('%SN%' % (self.num, val))

    def _wait_until_done(self):
        # wait_time = convert_to('seconds', on_dimensionless='warn')(self.wait_time)
        time.sleep(self.wait_time)
        while not self.motion_done:
            time.sleep(self.wait_time)
        return True
