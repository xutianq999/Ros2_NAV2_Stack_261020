"""Bounded action ownership, independent of command publication gaps."""


class RecoveryActivity:
    def __init__(self, timeout=25.0):
        # Exploration BT motions are capped at 20 s; allow 5 s for delivery/cancel.
        # This bound also releases ownership after a behavior server disappears.
        self.timeout = timeout
        self.goals = {}
        # Insertion-ordered set: evict the oldest finished goal first instead of
        # clearing everything (which could let a delayed status resurrect one).
        self.completed = {}

    def update(self, server, statuses, now):
        current = {}
        previous = self.goals.get(server, {})
        for item in statuses:
            if item.status in (1, 2, 3):  # accepted, executing, canceling
                key = bytes(item.goal_info.goal_id.uuid)
                if (server, key) in self.completed:
                    continue
                current[key] = previous.get(key, now)
        self.goals[server] = current

    def finish(self, server, key):
        self.goals.get(server, {}).pop(key, None)
        self.completed[(server, key)] = None
        # ROS status caches are finite; bound memory for long exploration runs.
        # Evict only the oldest entry; a delayed status for it is unlikely and
        # the 25 s lease still bounds the damage.
        if len(self.completed) > 256:
            self.completed.pop(next(iter(self.completed)))

    def server_gone(self, server):
        for key in list(self.goals.get(server, {})):
            self.finish(server, key)

    def count(self, now):
        # Repeated status messages must not extend a dead server's lease.
        return sum(0.0 <= now - start < self.timeout
                   for goals in self.goals.values() for start in goals.values())
