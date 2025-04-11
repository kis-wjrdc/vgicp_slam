from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            # ノードの基本情報
            package='vgicp_slam',          # パッケージ名
            executable='vgicp_slam_node',  # 実行ファイル名（CMakeLists.txt で設定したもの）
            name='vgicp_slam_node',        # ノード名（rqtやrvizで表示される名前）
            output='screen',               # 標準出力を画面に表示

            # パラメータ設定（すべてROS2パラメータとして宣言されている必要があります）
            parameters=[{
                # VGICP SLAM 全体の設定
                'voxel_size_for_vgicp': 0.3,    # VGICPに使う点群のダウンサンプリングボクセルサイズ
                'voxel_size_for_map': 0.1,      # 地図の保存用ボクセルサイズ
                'vgicp_resolution': 0.5,        # ボクセルの解像度（VGICP内部）
                'min_add_dist': 0.3,            # 地図に追加する最小距離（ロボットがこの距離以上動いたら更新）
                'window_size': 20,              # ローカルマップに保持するフレーム数

                # VGICPアルゴリズムの詳細設定（チューニング用）
                'vgicp_num_threads': 6,               # 並列計算に使うスレッド数
                'vgicp_max_iterations': 30,           # 最大反復回数
                'vgicp_correspondence_randomness': 30, # K近傍探索の数（多いほど精度↑、計算負荷↑）
                'vgicp_transformation_epsilon': 1.0e-5, # 平行移動の収束判定しきい値
                'vgicp_rotation_epsilon': 0.002        # 回転収束のしきい値（ラジアン）
                
            }]
        )
    ])
