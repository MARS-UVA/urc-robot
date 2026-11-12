# voice_control

Listens on the microphone, transcribes speech offline with [Vosk](https://alphacephei.com/vosk/),
and turns command words into ROS 2 action goals. The action interfaces are defined in
[`robot_actions`](../robot_actions).

```bash
ros2 run voice_control listener
```

Vosk isn't available through rosdep. Install it with `pip install vosk`; the English model
downloads the first time the node starts.

## Commands

Go to `voice_control/listener.py` to configure supported commands, words, and related actions.
