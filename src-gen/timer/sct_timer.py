from .interface_timer import ITimer
import threading
import time

class Timer(ITimer):

    """
    
    Default Timer for timed statecharts.
    
    """
    
    def __init__(self):
        self.is_default_timer = True
        self.timer_queue = {}
    
    def set_timer(self, callback, event_id, interval, periodic):
        """Schedule the time event.
        
        Checks if the given time event is already in queue and creates a
        timer based on the `periodic`parameter (Single or Repeat).
        
        Note: statemachine class converts time from sec to milliseconds,
        so do vice versa.
        """
        timer_id = (callback, event_id)
        print(f"[TIMER-DEBUG] set_timer called: event_id={event_id}, interval={interval}ms, periodic={periodic}")
        if timer_id in self.timer_queue:
            self.unset_timer(callback, event_id)
        m_interval = float(interval)/1000.0
        if periodic is False:
            self.timer_queue[timer_id] = SingleTimer(m_interval, callback, event_id)
        else:
            self.timer_queue[timer_id] = RepeatedTimer(m_interval, callback, event_id)
    
    def unset_timer(self, callback, event_id):
        """Cancel a certain event identified bei `callback` and `event_id`.
        """
        timer_id = (callback, event_id)
        with threading.RLock():
            if timer_id in self.timer_queue:
                event_timer = self.timer_queue[timer_id]
                if type(event_timer) is RepeatedTimer:
                    event_timer.stop()
                else:
                    event_timer.single_timer.cancel()
                del event_timer
    
    def cancel(self):
        """Cancel all events that are currently running.
        """
        with threading.RLock():
            for (callback, event_id) in self.timer_queue:
                self.unset_timer(callback, event_id)
    

class SingleTimer:

    """Call `function` after `period` seconds."""
    
    def __init__(self, period, callback, event_id):
        self.callback = callback
        self.event_id = event_id
        print(f"[TIMER-DEBUG] SingleTimer created: event_id={event_id}, period={period}s")
        self.single_timer = threading.Timer(period, self._fire)
        self.single_timer.start()
    
    def _fire(self):
        print(f"[TIMER-DEBUG] Timer FIRED: event_id={self.event_id}")
        self.callback.time_elapsed(self.event_id)

class RepeatedTimer:

    """Repeat `callback` every `interval` seconds."""
    
    def __init__(self, interval, callback, event_id):
        self.interval = interval
        self.callback = callback
        self.event_id = event_id
        self.start = time.time()
        self.event = threading.Event()
        self.thread = threading.Thread(target=self._target)
        self.thread.start()
        
    def _target(self):
        while not self.event.wait(self._time):
            self.callback.time_elapsed(self.event_id)
            
    @property
    def _time(self):
        return self.interval - ((time.time() - self.start) % self.interval)
        
    def stop(self):
        self.event.set()
        try:
            self.thread.join()
        except RuntimeError:
            print('Cannot join thread')
