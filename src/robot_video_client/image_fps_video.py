#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from threading import Lock
import time
from collections import defaultdict, deque

class ImageTopicMonitor(Node):
    def __init__(self):
        super().__init__('image_topic_monitor')

        # 要监控的话题列表
        self.topics = [
            '/camera_head_front/color/video',
            '/camera_head_front/depth/stream',
            '/left/color/video',
            '/left/left_ir/video',
            '/left/right_ir/video',
            '/left/depth/stream',
            '/right/color/video',
            '/right/left_ir/video',
            '/right/right_ir/video',
            '/right/depth/stream'
        ]

        # 存储每个话题的时间戳信息
        self.topic_timestamps = defaultdict(deque)  # 存储最近几个时间戳
        self.topic_last_time = {}  # 存储上一次收到消息的时间
        self.topic_message_count = defaultdict(int)  # 消息计数

        # 帧率计算相关
        self.topic_frame_timestamps = defaultdict(deque)  # 存储消息接收的绝对时间戳（用于计算帧率）
        self.fps_window_size = 30  # 用最近30帧计算帧率
        self.topic_fps = defaultdict(float)  # 存储每个话题的当前帧率

        # 线程锁
        self.lock = Lock()

        # 告警阈值 (秒)
        self.warning_threshold = 0.1  # 100ms

        # 创建订阅者
        self.subscribers = {}
        for topic in self.topics:
            self.subscribers[topic] = self.create_subscription(
                Image,
                topic,
                lambda msg, topic_name=topic: self.image_callback(msg, topic_name),
                10  # QoS队列大小
            )
            self.get_logger().info(f'订阅话题: {topic}')

        # 创建定时器，定期检查和报告状态
        self.timer = self.create_timer(5.0, self.status_report)  # 每5秒报告一次状态

        self.get_logger().info('图像话题监控器启动完成')

    def image_callback(self, msg, topic_name):
        """图像消息回调函数"""
        current_time = time.time()

        with self.lock:
            # 更新消息计数
            self.topic_message_count[topic_name] += 1

            # 添加当前时间戳到帧率计算队列
            self.topic_frame_timestamps[topic_name].append(current_time)

            # 保持帧率计算窗口大小
            if len(self.topic_frame_timestamps[topic_name]) > self.fps_window_size:
                self.topic_frame_timestamps[topic_name].popleft()

            # 计算帧率（如果有足够的数据点）
            self.calculate_fps(topic_name)

            # 检查时间间隔
            if topic_name in self.topic_last_time:
                time_interval = current_time - self.topic_last_time[topic_name]

                # 如果间隔大于阈值，发出告警
                if time_interval > self.warning_threshold:
                    current_fps = self.topic_fps.get(topic_name, 0.0)
                    self.get_logger().warn(
                        f'告警: 话题 {topic_name} 消息间隔过大: {time_interval:.3f}s (>{self.warning_threshold:.3f}s), '
                        f'当前帧率: {current_fps:.1f} FPS'
                    )

                # 保存最近的时间间隔（用于统计）
                self.topic_timestamps[topic_name].append(time_interval)

                # 只保留最近20个时间间隔
                if len(self.topic_timestamps[topic_name]) > 20:
                    self.topic_timestamps[topic_name].popleft()

            # 更新最后收到消息的时间
            self.topic_last_time[topic_name] = current_time

    def calculate_fps(self, topic_name):
        """计算指定话题的帧率"""
        timestamps = self.topic_frame_timestamps[topic_name]

        # 需要至少2个时间戳才能计算帧率
        if len(timestamps) < 2:
            self.topic_fps[topic_name] = 0.0
            return

        # 如果有足够的帧数，使用滑动窗口计算
        if len(timestamps) >= 5:  # 至少5帧才开始计算帧率
            time_span = timestamps[-1] - timestamps[0]  # 总时间跨度
            frame_count = len(timestamps) - 1  # 帧数间隔

            if time_span > 0:
                fps = frame_count / time_span
                self.topic_fps[topic_name] = fps
            else:
                self.topic_fps[topic_name] = 0.0
        else:
            # 数据不足时，暂时设为0
            self.topic_fps[topic_name] = 0.0

    def get_expected_fps_status(self, current_fps):
        """根据帧率返回状态描述"""
        if current_fps >= 25:
            return "优秀"
        elif current_fps >= 20:
            return "良好"
        elif current_fps >= 15:
            return "一般"
        elif current_fps >= 10:
            return "较低"
        elif current_fps > 0:
            return "过低"
        else:
            return "无数据"

    def status_report(self):
        """定期状态报告"""
        with self.lock:
            self.get_logger().info("=== 话题状态报告 ===")

            for topic in self.topics:
                if topic in self.topic_message_count:
                    msg_count = self.topic_message_count[topic]
                    current_fps = self.topic_fps.get(topic, 0.0)
                    fps_status = self.get_expected_fps_status(current_fps)

                    # 计算平均间隔
                    if topic in self.topic_timestamps and self.topic_timestamps[topic]:
                        intervals = list(self.topic_timestamps[topic])
                        avg_interval = sum(intervals) / len(intervals)
                        min_interval = min(intervals)
                        max_interval = max(intervals)

                        interval_status = "正常" if max_interval <= self.warning_threshold else "异常"

                        self.get_logger().info(
                            f'{topic}:\n'
                            f'  消息数: {msg_count}, 帧率: {current_fps:.1f} FPS [{fps_status}]\n'
                            f'  间隔统计: 平均{avg_interval:.3f}s, 最小{min_interval:.3f}s, '
                            f'最大{max_interval:.3f}s [{interval_status}]'
                        )
                    else:
                        self.get_logger().info(
                            f'{topic}:\n'
                            f'  消息数: {msg_count}, 帧率: {current_fps:.1f} FPS [{fps_status}]\n'
                            f'  间隔统计: 数据不足'
                        )
                else:
                    self.get_logger().warn(f'{topic}: 未收到任何消息, 帧率: 0.0 FPS [无数据]')

            # 添加总体统计
            total_messages = sum(self.topic_message_count.values())
            active_topics = len([t for t in self.topics if t in self.topic_message_count])
            avg_fps = sum(self.topic_fps.values()) / len(self.topic_fps) if self.topic_fps else 0.0

            self.get_logger().info(
                f'总体统计: {active_topics}/{len(self.topics)} 个话题活跃, '
                f'总消息数: {total_messages}, 平均帧率: {avg_fps:.1f} FPS'
            )
            self.get_logger().info("==================")

    def check_topic_timeout(self):
        """检查话题是否超时（可选功能）"""
        current_time = time.time()
        timeout_threshold = 1.0  # 1秒超时

        with self.lock:
            for topic in self.topics:
                if topic in self.topic_last_time:
                    time_since_last = current_time - self.topic_last_time[topic]
                    if time_since_last > timeout_threshold:
                        current_fps = self.topic_fps.get(topic, 0.0)
                        self.get_logger().error(
                            f'超时告警: 话题 {topic} 已经 {time_since_last:.1f}s 未收到消息, '
                            f'帧率: {current_fps:.1f} FPS'
                        )
                        # 超时时将帧率重置为0
                        self.topic_fps[topic] = 0.0

def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = ImageTopicMonitor()

        # 可以添加超时检查定时器（可选）
        timeout_timer = monitor.create_timer(2.0, monitor.check_topic_timeout)

        monitor.get_logger().info('开始监控图像话题...')
        rclpy.spin(monitor)

    except KeyboardInterrupt:
        print('\n收到中断信号，正在关闭...')
    except Exception as e:
        print(f'发生错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
