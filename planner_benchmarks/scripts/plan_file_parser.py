import re

def makespanFromPlan(plan):
    """ Plan is a list of (start, op, duration)
        Return the makespan of the plan. """
    latest_timestamp = -1
    for (st, op, dur) in plan:
        ts = st + dur
        if ts > latest_timestamp:
            latest_timestamp = ts
    assert latest_timestamp >= 0
    return latest_timestamp

def parsePlan(file):
    # Matches: start_time: (some action stuff) [duration]
    exp = "([0-9\\.]+) *: *\\(([a-zA-Z0-9\\-_ ]+)\\) *\\[([0-9\\.]+)\\]"
    with open(file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(";"):    # empty lines, comments start with ;
                continue
            m = re.match(exp, line)
            if not m:
                print "NO MATCH FOR", line
            start_time = (float)(m.group(1))
            op = m.group(2)
            duration = (float)(m.group(3))
            yield (start_time, op, duration)


