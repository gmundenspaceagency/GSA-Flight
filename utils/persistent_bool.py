class PersistentBool:
    def __init__(self, persist_duration=5):
        self.state = False
        self.counter = 0
        self.persist_duration = persist_duration

    def set_true(self):
        self.state = True

    def update(self):
        if self.state:
            self.counter += 1
            if self.counter >= self.persist_duration:
                self.state = False
                self.counter = 0
        else:
            self.counter = 0