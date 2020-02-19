# -*- coding: utf-8 -*-
"""
Donkeycar用Intel RealSense T265 トラッキングカメラパーツクラス。
Donkeycar v3.1.1 realsense2.py を書き換えたもの。
本コードもMITライセンス準拠とする。

1. donkeycar v3.1.1 をインストール
   https://github.com/autorope/donkeycar
2. librealsense をpythonラッパオプション付きでインストール
   https://github.com/IntelRealSense/librealsense
3. シャットダウンしT265をUSB(3.0推奨)接続してから再起動
4. 本コードを配置
5. manage.pyを修正
"""
import time
import logging
import math as m
import numpy as np
try:
    import pyrealsense2 as rs
except:
    print('[RealSenseT265] This module requires pyrealsense2 package!')
    raise

class RealSenseT265:
    '''
    The Intel Realsense T265 camera is a device which uses an imu, twin fisheye cameras,
    and an Movidius chip to do sensor fusion and emit a world space coordinate frame that 
    is remarkably consistent.
    '''

    def __init__(self, image_output=False, debug=False):
        """
        RealSense T265トラッキングカメラからデータを取得するパーツクラス。
        引数：
            image_output    T265に搭載された2つの魚眼カメラのうち片方から
                            画像ストリームを取得する(USB3.0推奨)。
                            デフォルトはFalse、runを実行すると常にNoneが返却される。
            debug           デバッグフラグ。真値にすると標準出力にログを出力する。
        戻り値：
            なし
        """
        self.debug = debug
        self.image_output = image_output

        # RealSenseパイプラインを宣言し、実際のデバイスとセンサをカプセル化
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        if self.image_output:
            # 現時点で両カメラを有効にする必要あり
            cfg.enable_stream(rs.stream.fisheye, 1) # 左カメラ
            cfg.enable_stream(rs.stream.fisheye, 2) # 右カメラ

        # 要求された校正でストリーミング開始
        self.pipe.start(cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.ang = zero_vec
        self.img = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            if self.debug:
                print(e)
            logging.error(e)
            return

        if self.image_output:
            # 左魚眼ガメラからイメージを取得する
            left = frames.get_fisheye_frame(1)
            self.img = np.asanyarray(left.get_data())


        # 位置情報フレームをフェッチ
        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            print(type(data))
            priint(data)
            self.pos = (data.translation.x, data.translation.y, data.translation.z)
            self.vel = (data.velocity.x, data.velocity.y, data.velocity.z)
            self.acc = (data.acceleration.x, data.acceleration.y, data.acceleration.z)
            # 四元数
            w = data.rotation.w
            x = -data.rotation.z
            y = data.rotation.x
            z = -data.rotation.y
            # Eular Angle
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi
            pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi
            self.ang = (roll, pitch, yaw)
            logging.debug('[RealSenseT265] poll() pos(%f, %f, %f)' % (self.pos[0], self.pos[1], self.pos[2]))
            logging.debug('[RealSenseT265] poll() ang(%f, %f, %f)' % (self.ang[0], self.ang[1], self.ang[2]))
            if self.debug:
                print('[RealSenseT265] poll() pos(%f, %f, %f)' % (self.pos[0], self.pos[1], self.pos[2]))
                print('[RealSenseT265] poll() ang(%f, %f, %f)' % (self.ang[0], self.ang[1], self.ang[2]))

    def update(self):
        """
        別スレッドが生成されたら、このメソッドが呼び出される。
        T265からセンサデータを取得する。
        インスタンス変数runningが真である間、poll()を実行する。
        引数：
            なし
        戻り値：
            なし
        """
        while self.running:
            self.poll()

    def run_threaded(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが真である場合、
        run()のかわりに呼び出される。
        引数：
            なし
        戻り値：
            pos_x       位置情報X軸
            pos_y       位置情報Y軸
            pos_z       位置情報Z軸
            vel_x       速度X軸
            vel_y       速度Y軸
            vel_z       速度Z軸
            acc_x       加速度X軸
            acc_y       加速度Y軸
            acc_z       加速度Z軸
            ang_x       Eular Angle X軸(ロール)
            ang_y       Eular Angle Y軸(ピッチ)
            ang_z       Eular Angle Z軸(ヨー)
            image_array 左カメライメージ(nd.array型、(800,848)形式)
        """
        return self.pos[0], self.pos[1], self.pos[2], \
            self.vel[0], self.vel[1], self.vel[2], \
            self.acc[0], self.acc[1], self.acc[2], \
            self.ang[0], self.ang[1], self.ang[2], \
            self.img

    def run(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが偽である場合、
        run_threaded()のかわりに呼び出される。
        引数：
            なし
        戻り値：
            pos_x       位置情報X軸
            pos_y       位置情報Y軸
            pos_z       位置情報Z軸
            vel_x       速度X軸
            vel_y       速度Y軸
            vel_z       速度Z軸
            acc_x       加速度X軸
            acc_y       加速度Y軸
            acc_z       加速度Z軸
            ang_x       Eular Angle X軸(ロール)
            ang_y       Eular Angle Y軸(ピッチ)
            ang_z       Eular Angle Z軸(ヨー)
            image_array 左カメライメージ(nd.array型、(800,848)形式)
        """
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        """
        パーツクラスTemplate Methodのひとつ。終了時処理。
        マルチスレッドループを閉じ、T265パイプを停止する。
        引数：
            なし
        戻り値：
            なし
        """
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()



if __name__ == "__main__":
    c = RealSenseT265(image_output=True, debug=True)
    while True:
        c.run()
        #print(pos)
        time.sleep(0.1)
    c.shutdown()
