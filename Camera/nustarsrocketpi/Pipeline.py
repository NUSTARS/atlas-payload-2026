import threading
import queue

SENTINEL = object()

class Pipeline(threading.Thread):
    def __init__(self, threadID, name, counter, *,
                 stages,
                 in_queue: "queue.Queue",
                 out_queue: "queue.Queue|None" = None,
                 daemon: bool = True):
        super().__init__(name=name, daemon=daemon)
        self.threadID = threadID
        self.counter = counter
        self.stages = stages
        self.in_q = in_queue
        self.out_q = out_queue
        self.stop_event = threading.Event()
        pass

    def feed(self, fb):
        pass

    def stop(self):
        pass

    def run(self):
        pass
