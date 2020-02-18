# -*- coding: utf-8 -*-
"""
Donkeycar用Intel RealSense T265 トラッキングカメラパーツクラス。
カメラ画像をVehicleフレームワーク上で使用する場合は、image_outputを
真にしてコンストラクタを呼び出す。
ただしイメージ配列は(800,848)形式のnd.array型であり、そのままでは
Donkeycarのcam/image_arrayの代替として使用することはできないことに注意。
要librealsenseインストール(ビルド時要-DBUILD_PYTHON_BINDINGS:bool=true)。
"""
import time
import math as m
import numpy as np
try:
    import pyrealsense2 as rs
except:
    print('[RealSenseT265] This module requires pyrealsense2 package!')
    raise

class RealSenseT265:

    def __init__(self, image_output=False, calib_filename=None, debug=False):
        """
        RealSense T265トラッキングカメラからデータを取得するパーツクラス。
        引数：
            image_output    T265に搭載された2つの魚眼カメラのうち片方から
                            画像ストリームを取得する(USB3.0推奨)。
                            デフォルトはFalse、runを実行すると常にNoneが返却される。
            calib_filename　ホィールオドメトリキャリブレーションファイル名を指定する。
                            指定しない場合はキャリブレーションしない。
            debug           デバッグフラグ。真値にすると標準出力にログを出力する。
        戻り値：
            なし
        """
        self.debug = debug
        self.image_output = image_output

        # エンコーダがある場合、測定される最後の速度になる
        self.enc_vel_ms = 0.0
        self.wheel_odometer = None

        # RealSenseパイプラインを宣言し、実際のデバイスとセンサをカプセル化する
        if self.debug:
            print('[RealSenseT265] starting T265')
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        profile = cfg.resolve(self.pipe)
        dev = profile.get_device()
        tm2 = dev.as_tm2()
        

        if self.image_output:
            # 現時点で両カメラを有効にする必要あり
            cfg.enable_stream(rs.stream.fisheye, 1) # 左カメラ
            cfg.enable_stream(rs.stream.fisheye, 2) # 右カメラ

        # ホィールオドメトリによる補正を行う場合
        if calib_filename is not None:
            pose_sensor = tm2.first_pose_sensor()
            self.wheel_odometer = pose_sensor.as_wheel_odometer() 

            # ホィールオドメトリキャリブレーションファイルデータをuint8リスト化
            f = open(calib_filename)
            chars = []
            for line in f:
                for c in line:
                    chars.append(ord(c))  # char から uint8 へ

            # ホィールオドメータをロード/設定
            if self.debug:
                print("[RealSenseT265] loading wheel config", calib_filename)
            self.wheel_odometer.load_wheel_odometery_config(chars)   


        # 要求された校正でストリーミング開始
        self.pipe.start(cfg)
        self.running = True
        if self.debug:
            print("[RealSenseT265] Warning: T265 needs a warmup period of a few seconds before it will emit tracking data.")
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.ang = zero_vec
        self.img = None

    def poll(self):
        """
        T265からデータを取得し、インスタンス変数へ格納する。
        引数：
            なし
        戻り値：
            なし
        """
        #  ホィールオドメータを使用する場合
        if self.wheel_odometer:
            wo_sensor_id = 0  # 0から開始、キャリブレーションファイルの順序に一致
            frame_num = 0  # 使用しない
            v = rs.vector()
            v.x = -1.0 * self.enc_vel_ms  # m/秒
            #v.z = -1.0 * self.enc_vel_ms  # m/秒
            self.wheel_odometer.send_wheel_odometry(wo_sensor_id, frame_num, v)

        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            print(e)
            return

        # カメライメージをrunの戻り値として使用する場合
        if self.image_output:
            # 左魚眼ガメラからイメージを取得する
            left = frames.get_fisheye_frame(1)
            self.img = np.asanyarray(left.get_data())


        # 位置情報フレームをフェッチ
        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
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

    def run_threaded(self, enc_vel_ms):
        """
        パーツクラスTemplate Methodのひとつ。threadedが真である場合、
        run()のかわりに呼び出される。
        引数：
            enc_vel_ms  エンコーダの速度(単位:m/秒)
                        エンコーダを使用しない場合は固定値0.0を指定のこと。
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
        if enc_vel_ms is not None:
            self.enc_vel_ms = enc_vel_ms
        return self.pos[0], self.pos[1], self.pos[2], \
            self.vel[0], self.vel[1], self.vel[2], \
            self.acc[0], self.acc[1], self.acc[2], \
            self.ang[0], self.ang[1], self.ang[2], \
            self.img

    def run(self, enc_vel_ms):
        """
        パーツクラスTemplate Methodのひとつ。threadedが偽である場合、
        run_threaded()のかわりに呼び出される。
        引数：
            enc_vel_ms  エンコーダの速度(単位:m/秒)。
                        エンコーダを使用しない場合は固定値0.0を指定のこと。
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
        if enc_vel_ms is not None:
            self.enc_vel_ms = enc_vel_ms
        self.poll()
        return self.run_threaded(enc_vel_ms)

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
        if self.debug:
            print('[RealSenseT265] shutdown() done')



if __name__ == "__main__":
    """
    疎通確認用処理
    """
    c = RealSenseT265(image_output=True, debug=True)
    while True:
        c.run(0.0)
        time.sleep(0.1)
    c.shutdown()