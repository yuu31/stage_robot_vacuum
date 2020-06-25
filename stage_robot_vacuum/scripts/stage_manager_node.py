#! /usr/bin/env python
# -*- coding: utf-8 -*-

import random
import time

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

#定義変数
COMPETITION_TIME = 180.0

class stage_manager:
    def __init__(self):
        rospy.init_node("stage_manager")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.checkCallback)
        rospy.Subscriber("/check_signal", Bool, self.cs_Callback)
        self.point_pub = rospy.Publisher("/check_point", Point, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        rospy.Timer(rospy.Duration(1.0), self.timerCallback2)

        # 通過ポイントのリスト
        self.c_point_list = [[-6.65,1.3],[1.25, -1.25],
                             [3.75,6.25],[6.25,-6.25]]

        self.c_point_list2 = [[-8.75, 8.75],[-6.25, -3.75],[-1.25, 6.25],[1.25, -3.75],
                              [3.75, 3.75],[8.75, -1.25],[8.75, -8.75]]
        # ポイント1からランダムに2つ選択
        self.c_point_list = random.sample(self.c_point_list, 2)
        # 通過ポイント2からランダムに4つ選択
        self.r_c_point_list2 = random.sample(self.c_point_list2, 7)
        self.c_point_list2_True = self.r_c_point_list2[0:4]
        self.c_point_list2_False = self.r_c_point_list2[4:7]
        # 通過ポイントとロボット座標との距離の閾値
        self.dis = 1.0
        self.topic_dis = 3.0
        # 通過ポイントのカウント用
        self.point_flag = [False] * len(self.c_point_list)
        self.point_flag2 = [False] * len(self.c_point_list2_True)
        self.point_rt_flag = [False] * len(self.c_point_list2_True)
        self.point_rt_flag_False = [False] * len(self.c_point_list2_False)

        # メッセージの型はgeometry_msgs/Point
        self.point = Point()
        self.point.x = 0.0
        self.point.y = 0.0
        self.point.z = 0.0

        goal_list = [[-8.75, 6.25],[3.75, 8.75],[6.25, -8.75]]
        #ゴールをランダムで決定
        rnum = random.randrange(len(goal_list))

        # ゴール座標を設定
        self.goal = Point()
        self.goal.x = goal_list[rnum][0]
        self.goal.y = goal_list[rnum][1]
        self.goal.z = 0.0

        #競技時間の計測開始
        self.start = time.time()
        self.end_flag = False

        # 点数の設定
        self.score = 0.0
        self.time_score = 0.0
        self.p1_score = 10
        self.p2_score = 30
        self.miss_score = 0.0

    def checkCallback(self, data):
        position = data.pose.pose.position
        l = len(self.c_point_list)
        l2 = len(self.c_point_list2_True)
        l3 = len(self.c_point_list2_False)

        # ロボット座標とすべての通過ポイントの差分を確認
        for i in range(l):
            diff_x = abs(self.c_point_list[i][0] - position.x)
            diff_y = abs(self.c_point_list[i][1] - position.y)
            # ユークリッド距離を計算
            diff = pow(diff_x**2 + diff_y**2, 0.5)
            # 配信位置とロボット位置が閾値より近い時，通過ポイントの座標を配信
            if diff < self.topic_dis:
                self.point.x = self.c_point_list[i][0]
                self.point.y = self.c_point_list[i][1]
                self.point_pub.publish(self.point)

        # ロボット座標とすべての通過ポイントの差分を確認
        for i in range(l):
            diff_x = abs(self.c_point_list[i][0] - position.x)
            diff_y = abs(self.c_point_list[i][1] - position.y)
            # ユークリッド距離を計算
            diff = pow(diff_x**2 + diff_y**2, 0.5)
            # 通過ポイントとロボット位置が閾値より近い
            if diff < self.dis:
                # 通過ポイントを記録
                self.point_flag[i] = True

        # ロボット座標と通過ポイント2の差分を確認
        for i in range(l2):
            diff_x = abs(self.c_point_list2_True[i][0] - position.x)
            diff_y = abs(self.c_point_list2_True[i][1] - position.y)

            # ユークリッド距離を計算
            diff = pow(diff_x**2 + diff_y**2, 0.5)
            # 現時点での通過ポイントを記録
            if diff < self.dis:
                self.point_rt_flag[i] = True

            else:
                self.point_rt_flag[i] = False

        # ロボット座標とはずれ通過ポイント2の差分を確認
        for i in range(l3):
            diff_x = abs(self.c_point_list2_False[i][0] - position.x)
            diff_y = abs(self.c_point_list2_False[i][1] - position.y)

            # ユークリッド距離を計算
            diff = pow(diff_x**2 + diff_y**2, 0.5)
            # 現時点での通過ポイントを記録
            if diff < self.dis:
                self.point_rt_flag_False[i] = True
            else:
                self.point_rt_flag_False[i] = False

        # ゴールとの距離を計算
        diff_x = abs(self.goal.x - position.x)
        diff_y = abs(self.goal.y - position.y)
        diff = pow(diff_x**2 + diff_y**2, 0.5)
        # ゴールしたら終了
        if diff < self.dis:
            if(self.end_flag == False):
                print("ゴール！！")
                # ゴール点数
                self.score += 100
                # 残り時間
                self.score += self.time_score
                # 通過ポイント * a 点加算
                self.score += sum(self.point_flag) * self.p1_score
                self.score += sum(self.point_flag2) * self.p2_score
                self.score += self.miss_score

                print("得点:{0:0f}点".format(self.score))
                self.end_flag = True

    #チェックポイント2通過の判定
    def cs_Callback(self, signal):
        # シグナルがONと配信された場合の処理
        if (signal.data == True):
            l = len(self.c_point_list2_True)
            if (sum(self.point_rt_flag) > 0):
                for i in range(l):
                    # 通過ポイント2にロボットが接近している場合の処理
                    if (self.point_rt_flag[i] == True):
                        # 通過ポイントを記録
                        self.point_flag2[i] = True

            # 外れポイントだった時
            elif (sum(self.point_rt_flag_False) > 0):
                pass
            else:
                # シグナルONであるが，通過ポイント2と離れている場合の処理
                self.miss_score -= -10

        else:
            pass

    def timerCallback(self, event):
        # ゴール座標を配信
        self.goal_pub.publish(self.goal)

    #競技時間計測用タイマー
    #1秒単位で実行
    def timerCallback2(self, event):
        #競技の残り時間の計算
        elapsed_time = time.time() - self.start
        remaining_time = COMPETITION_TIME - elapsed_time
        self.time_score = remaining_time

        if(remaining_time > 0):
            if(self.end_flag == False):
                print("残り時間:{0:.0f}秒".format(remaining_time))
        else:
            if(self.end_flag == False):
                print("終了")
                # 通過ポイント * a 点加算
                self.score += sum(self.point_flag) * self.p1_score
                self.score += sum(self.point_flag2) * self.p2_score
                self.score += self.miss_score

                print("得点:{0:0f}点".format(self.score))
                self.end_flag = True


if __name__ == '__main__':
    try:
        cp = stage_manager()
        rospy.spin()
    except rospy.ROSInterruptException: pass
