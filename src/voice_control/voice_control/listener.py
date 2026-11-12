import json

import pyaudio
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from vosk import KaldiRecognizer, Model

from robot_actions.action import Come, Fetch, Follow, Give, Stay

DEBUG = False

RATE = 16000  # Vosk models are trained on 16 kHz audio
CHUNK = 4000  # 0.25 s per buffer

# keyword -> (action type, action server name)
WORD_ACTION_MAP = {
    'follow': (Follow, 'follow'),
    'give': (Give, 'give'),
    'stay': (Stay, 'stay'),
    'fetch': (Fetch, 'fetch'),
    'come': (Come, 'come'),
}


class VoiceListener(Node):

    def __init__(self):
        super().__init__('listener')
        # raw transcripts are only published for debugging
        self.final_pub = (self.create_publisher(String, 'voice/final', 10)
                          if DEBUG else None)

        self.action_clients = {
            word: ActionClient(self, action_type, server_name)
            for word, (action_type, server_name) in WORD_ACTION_MAP.items()
        }
        self.active_goal = None  # ClientGoalHandle of the running action
        self.request_id = 0  # basically a counter

        self.rec = KaldiRecognizer(Model(lang='en-us'), RATE)
        self.rec.SetWords(True)

        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(format=pyaudio.paInt16, channels=1, rate=RATE,
                                   input=True, frames_per_buffer=CHUNK,
                                   stream_callback=self._audio_callback)
        self.stream.start_stream()
        self.get_logger().info('Listening...')

    def _audio_callback(self, in_data, frame_count, time_info, status):
        # if the recognizer is slow, buffer will have too much information
        if status & pyaudio.paInputOverflow:
            self.get_logger().warn('Input overflow: recognizer too slow',
                                   throttle_duration_sec=2.0)
        if self.rec.AcceptWaveform(in_data):
            text = json.loads(self.rec.Result())['text']
            if text:
                self.get_logger().info(f'Heard: "{text}"')
                if self.final_pub is not None:
                    self.final_pub.publish(String(data=text))

                word = next((w for w in text.split() if w in WORD_ACTION_MAP), None)
                if word:
                    self._send_action(word)

        return (None, pyaudio.paContinue)

    def _send_action(self, word):
        client = self.action_clients[word]
        if not client.server_is_ready():
            return

        # a new command replaces whatever the robot is currently doing
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
            self.active_goal = None

        self.request_id += 1
        request_id = self.request_id
        action_type, _ = WORD_ACTION_MAP[word]
        client.send_goal_async(action_type.Goal()).add_done_callback(
            lambda f: self._on_goal_response(word, request_id, f))

    def _on_goal_response(self, word, request_id, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'"{word}" goal rejected')
            return
        # a newer command was issued while this goal was in flight
        if request_id != self.request_id:
            self.get_logger().info(f'"{word}" superseded, cancelling')
            handle.cancel_goal_async()
            return
        self.get_logger().info(f'"{word}" goal accepted')
        self.active_goal = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(handle, f))

    def _on_result(self, handle, future):
        if self.active_goal is handle:
            self.active_goal = None

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        for client in self.action_clients.values():
            client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
