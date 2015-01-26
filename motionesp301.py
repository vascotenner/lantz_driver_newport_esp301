# -*- coding: utf-8 -*-
"""
    lantz.drivers.newport.motionesp301
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    Implements the drivers to control an ESP301 motion controller via USB or serial.

    For USB, one first have to install the windows driver from newport.

    :copyright: 2014 by Lantz Authors, see AUTHORS for more details.
    :license: BSD, see LICENSE for more details.
    
    Source: Instruction Manual (Newport)
"""


from lantz.feat import Feat
from lantz.action import Action
from lantz.serial import SerialDriver
from lantz.visa import GPIBVisaDriver
from lantz import Q_, ureg
from lantz.processors import convert_to
import time

# Add generic units:
#ureg.define('unit = unit')
#ureg.define('encodercount = count')
#ureg.define('motorstep = step')


class ESP301():
    """ Newport ESP301 motion controller. It assumes all axes to have units mm"""

    RECV_TERMINATION = '\r\n'
    SEND_TERMINATION = '\r\n'

    def initialize(self):
        super().initialize()

        self.axes = []
        self.detect_axis()

    @Action()
    def detect_axis(self):
        """ Find the number of axis available """
        i = 0
        self.axes = []
        while self.scan_axes:
            try:
                i += 1
                id = self.query('%dID?' % i)
                axis = ESP301Axis(self, i, id)
                self.axes.append(axis)
            except:
                err = int(self.query('TE?'))
                if err == 37:  # Axis number missing
                    self.axes.append(None)
                elif err == 9:  # Axis number out of range
                    self.scan_axes = False
                else:  # Dunno...
                    raise

    def finalize(self):
        for axis in self.axes:
            if axis is not None:
                del (axis)
        super().finalize()


class ESP301USB(ESP301,SerialDriver):
    """ Newport ESP301 motion controller. It assumes all axes to have units mm

    :param scan_axes: Should one detect and add axes to the controller
    :param port: Which serial port should be used. 0 for Serial (COM1), 2 for USB (COM3)
    :param usb: True if connection is made by USB
    :param *args, **kwargs: Passed to lantz.serial.SerialDriver
    """

    ENCODING = 'ascii'
    TIMEOUT = 4

    BAUDRATE = 19200
    BYTESIZE = 8
    PARITY = 'none'
    STOPBITS = 1

    #: flow control flags
    RTSCTS = False
    DSRDTR = False
    XONXOFF = False


    def __init__(self, scan_axes=True, port=0, usb=True, *args, **kwargs):
        if usb:
            self.BAUDRATE = 921600
        else:
            self.BAUDRATE = 19200

        super().__init__(port, 4, write_timeout=4, *args, **kwargs)

        self.scan_axes = scan_axes


class ESP301GPIB( ESP301, GPIBVisaDriver):
    """ Untested!
    """
    def __init__(self, scan_axes=True, resource_name= 'GPIB0::2::INSTR', *args, **kwargs):
        # Read number of axes and add axis objects
        self.scan_axes = scan_axes
        super().__init__(resource_name=resource_name, *args, **kwargs)


class ESP301Axis(SerialDriver,object):
    def __init__(self, parent, num, id, *args, **kwargs):
        super(ESP301Axis, self).__init__(*args, **kwargs)
        self.parent = parent
        self.num = num
        self.id = id
        self.wait_time = 0.1 # in seconds * Q_(1, 's')
        self.backlash = 0
        self.wait_until_done = True

    def __del__(self):
        self.parent = None
        self.num = None

    def id(self):
        return self.id

    @Action()
    def on(self):
        """Put axis on"""
        self.parent.send('%dMO' % self.num)

    @Action()
    def off(self):
        """Put axis on"""
        self.parent.send('%dMF' % self.num)

    @Feat(values={True: '1', False: '0'})
    def is_on(self):
        """
        :return: True is axis on, else false
        """
        return self.parent.query('%dMO?' % self.num)

    @Action(units='mm')
    def define_home(self, val=0):
        """Remap current position to home (0), or to new position

        :param val: new position"""
        self.parent.send('%dDH%f' % (self.num, val))

    @Feat(units='mm')
    def position(self):
        return float(self.parent.query('%dTP?' % self.num))

    @position.setter
    def position(self, pos):
        """
        Waits until movement is done if self.wait_until_done = True.

        :param pos: new position
        """

        # First do move to extra position if necessary
        self._set_position(pos)

    @Action(units=['mm',None])
    def _set_position(self, pos, wait=None):
        """
        Move to an absolute position, taking into account backlash.

        When self.backlash is to a negative value the stage will always move
         from low to high values. If necessary, a extra step with length
         self.backlash is set.

        :param pos: New position in mm
        :param wait: wait until stage is finished
        """
        if wait is None:
            wait = self.wait_until_done

        # First do move to extra position if necessary
        position = self.position
        if ( self.backlash < 0 and position > pos) or\
                ( self.backlash > 0 and position < pos):
            self.__set_position(pos + self.backlash)
            if wait:
                self._wait_until_done()

        # Than move to final position
        self.__set_position(pos)
        if wait:
            self._wait_until_done()

    def __set_position(self, pos):
        """
        Move stage to a certain position
        :param pos: New position
        """
        self.parent.send('%dPA%f' % (self.num, pos))

    @Feat(units='mm/s')
    def max_velocity(self):
        return float(self.parent.query('%dVU?' % self.num))

    @max_velocity.setter
    def max_velocity(self, velocity):
        self.parent.send('%dVU%f' % (self.num, velocity))

    @Feat(units='mm/s')
    def velocity(self):
        return float(self.parent.query('%dVA?' % self.num))

    @velocity.setter
    def velocity(self, velocity):
        """
        :param velocity: Set the velocity that the axis should use when moving
        :return:
        """
        self.parent.send('%dVA%f' % (self.num, velocity))

    @Feat(units='mm/s')
    def actual_velocity(self):
        return float(self.parent.query('%dTV' % self.num))

    @actual_velocity.setter
    def actual_velocity(self, val):
        raise NotImplementedError

    @Action()
    def stop(self):
        """Emergency stop"""
        self.parent.send(u'{0:d}ST'.format(self.num))

    @Feat(values={True: '1', False: '0'})
    def motion_done(self):
        return self.parent.query('%dMD?' % self.num)

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
    def units(self):
        ret =  int(self.parent.query(u'{}SN?'.format(self.num)))
        vals = {0 :'encoder count',
                 1 :'motor step',
                 2 :'millimeter',
                 3 :'micrometer',
                 4 :'inches',
                 5 :'milli-inches',
                 6 :'micro-inches',
                 7 :'degree',
                 8 :'gradian',
                 9 :'radian',
                 10:'milliradian',
                 11:'microradian',}
        return vals[ret]

    # @units.setter
    # def units(self, val):
    #     self.parent.send('%SN%' % (self.num, val))

    def _wait_until_done(self):
        #wait_time = convert_to('seconds', on_dimensionless='warn')(self.wait_time)
        while not self.motion_done:
            time.sleep(self.wait_time) #wait_time.magnitude)



if __name__ == '__main__':
    import argparse
    import lantz.log

    parser = argparse.ArgumentParser(description='Test ESP301 driver')
    parser.add_argument('-p', '--port', type=str, default='1',
                        help='Serial port to connect to')

    args = parser.parse_args()
    lantz.log.log_to_socket(lantz.log.DEBUG)

    with ESP301(args.port) as inst:
        # inst.initialize() # Initialize the communication with the power meter
        # Find the status of all axes:
        for axis in inst.axes:
            print('Axis {} Position {} is_on {} max_velocity {} velocity {}'.format(axis.num, axis.position,
                                                                                    axis.is_on, axis.max_velocity,
                                                                                    axis.velocity))
