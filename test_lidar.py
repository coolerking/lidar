# -*- coding: utf-8 -*-
from time import sleep
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


    '''
    from donkeycar.parts.lidar import LidarPlot
    lidarplot = LidarPlot()
    V.add(lidarplot, inputs=['distances', 'angles'], outputs=['frame'])
    '''

    '''
    class PrintLidar(object):
        def run(self, frame):
            print('[LidarPlot] frame:{}'.format(str(type(frame))))
            print('[LidarPlot] frame:{}'.format(str(frame)))
        def shutdown(self):
            pass
    V.add(PrintLidar(), inputs=['frame'])
    '''

    '''
    from donkeycar.parts.lidar import MapToImage
    m2i = MapToImage()
    V.add(m2i, inputs=['frame'], outputs=['img_array'])
    '''

    from donkeycar.parts.lidar import BreezySLAM
    slam = BreezySLAM()
    V.add(slam, inputs=['distances', 'angles', 'map_bytes'], outputs=['x', 'y', 'rad'])
    
    class PrintB:
        def run(self, x, y, rad):
            print('[BreezySLAM] x:{}, y:{}, rad:{}'.format(str(x), str(y), str(rad)))
        def shutdown(self):
            pass
    V.add(PrintB(), inputs=['x', 'y', 'rad'])

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('exit')
    finally:
        pass

if __name__ == '__main__':
    cfg = dk.load_config()
    test_lidar(cfg)