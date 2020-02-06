# -*- coding: utf-8 -*-
from time import sleep
import math
import donkeycar as dk


def test_rs(cfg):

    V = dk.vehicle.Vehicle()

    from donkeycar.parts.realsense2 import RS_T265
    rs = RS_T265()
    V.add(rs, outputs=['pos', 'vel', 'acc', 'image_array'], threaded=False)

    class Split(object):
        def run(self, pos, vel, acc):
            return pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z
        def shutdown(self):
            pass
    V.add(Split(), inputs=['pos', 'vel', 'acc'],
        outputs=['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'acc_x', 'acc_y', 'acc_z'])

    class PrintRS(object):
        def run(self, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z):
            print('[RS] pos:({}, {}, {}) vel:({}, {}, {}) acc:({}, {}, {})'.format(
                str(pos_x), str(pos_y), str(pos_z),
                str(vel_x), str(vel_y), str(vel_z),
                str(acc_x), str(acc_y), str(acc_z)))
        def shutdown(self):
            pass
    V.add(PrintRS(), inputs=['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'acc_x', 'acc_y', 'acc_z'])





    inputs=['image_array',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'acc_x', 'acc_y', 'acc_z', 
            'rad']

    types=['image_array',
           'float', 'float', 'float',
           'float', 'float', 'float',
           'float', 'float', 'float',
           'float']

    from donkeycar.parts.datastore import TubHandler
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=[])
    V.add(tub, inputs=inputs, outputs=["tub/num_records"])

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.DRIVE_LOOP_HZ * 60) # 1min
            ## max_loop_count=cfg.MAX_LOOPS) # infinity
    except KeyboardInterrupt:
        print('exit')
    finally:
        pass

if __name__ == '__main__':
    cfg = dk.load_config()
    test_rs(cfg)