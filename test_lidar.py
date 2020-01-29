# -*- coding: utf-8 -*-
from time import sleep
import donkeycar as dk


def test_lidar(cfg):

    V = dk.vehicle.Vehicle()

    from donkeycar.parts.lidar import BreezySLAM, RPLidar, LidarPlot, MapToImage

    rplidar = RPLidar()
    V.add(rplidar, outputs=['distances', 'angles'], threaded=True)

    lidarplot = LidarPlot()
    V.add(lidarplot, inputs=['distances', 'angles'], outputs=['frame'])

    m2i = MapToImage()
    V.add(m2i, inputs=['frame'], outputs=['img_array'])

    slam = BreezySLAM()
    V.add(slam, inputs=['distances', 'angles', 'img_array'], outputs=['x', 'y', 'rad'])



    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('exit')
    finally:
        pass

if __name__ == '__main__':
    cfg = dk.load_config()
    test_lidar(cfg)