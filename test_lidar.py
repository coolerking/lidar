# -*- coding: utf-8 -*-
from time import sleep
import math
import donkeycar as dk


def test_lidar(cfg):

    V = dk.vehicle.Vehicle()

    from donkeycar.parts.lidar import BreezySLAM, LidarPlot, MapToImage

    from donkeycar.parts.lidar import RPLidar
    rplidar = RPLidar()
    V.add(rplidar, outputs=['distances', 'angles'], threaded=True)

    '''
    class PrintLidar(object):
        def run(self, distances, angles):
            print('[RPLidar] d:{} a:{}'.format(str(distances), str(angles)))
            print('[RPLidar] d:{} a:{}'.format(str(len(distances)), str(len(angles))))
        def shutdown(self):
            pass
    V.add(PrintLidar(), inputs=['distances', 'angles'])
    '''

    from donkeycar.parts.lidar import BreezyMap
    bmap = BreezyMap()
    V.add(bmap, outputs=['map_bytes'])

    from donkeycar.parts.lidar import LidarPlot
    lidarplot = LidarPlot()
    V.add(lidarplot, inputs=['distances', 'angles'], outputs=['frame'])

    '''
    class PrintLidar(object):
        def run(self, frame):
            print('[LidarPlot] frame:{}'.format(str(type(frame))))
            print('[LidarPlot] frame:{}'.format(str(frame)))
        def shutdown(self):
            pass
    V.add(PrintLidar(), inputs=['frame'])
    '''

    from donkeycar.parts.lidar import MapToImage
    m2i = MapToImage()
    V.add(m2i, inputs=['frame'], outputs=['image_array'])


    from donkeycar.parts.lidar import BreezySLAM
    slam = BreezySLAM()
    V.add(slam, inputs=['distances', 'angles', 'map_bytes'], outputs=['x', 'y', 'rad'])
    
    class PrintB:
        def run(self, x, y, rad):
            degree = math.degrees(rad)
            if degree >= 180.0:
                degree = (360.0 - degree) * (-1.0)
            xcm = float(x)/10.0
            ycm = float(y)/10.0
            print('{:.5g},{:.5g},{:.5g}'.format(xcm, ycm, degree))
            #print('[BreezySLAM] x:{:.3g}cm , y:{:.3g}cm , theta:{:.3g}degree(s)'.format(xcm, ycm, degree))
        def shutdown(self):
            pass
    V.add(PrintB(), inputs=['x', 'y', 'rad'])


    inputs=['image_array',
            'x', 'y', 
            'rad']

    types=['image_array',
           'float', 'float',
           'float']

    from donkeycar.parts.datastore import TubHandler
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=[])
    V.add(tub, inputs=inputs, outputs=["tub/num_records"])

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('exit')
    finally:
        pass

if __name__ == '__main__':
    cfg = dk.load_config()
    test_lidar(cfg)