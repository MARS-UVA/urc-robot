from vosk import Model, KaldiRecognizer
import pyaudio
import json


RATE = 16000  # Vosk models are trained on 16 kHz audio
CHUNK = 4000  # 0.25 s per read

model = Model(lang="en-us")
rec = KaldiRecognizer(model, RATE)
rec.SetWords(True)

pa = pyaudio.PyAudio()
stream = pa.open(format=pyaudio.paInt16, channels=1, rate=RATE,
                 input=True, frames_per_buffer=CHUNK)

try:
    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        if rec.AcceptWaveform(data):
            print("FINAL:", json.loads(rec.Result()))
        else:
            print("partial:", json.loads(rec.PartialResult())["partial"], end="\r")
finally:
    print(json.loads(rec.FinalResult())["text"])
    stream.stop_stream(); stream.close(); pa.terminate()
