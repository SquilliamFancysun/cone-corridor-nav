# Output of the topology layer: discrete events the global layer acts on.
std_msgs/Header header

uint8 EVENT_GATE_IN_RANGE=0   # blue pair sighted ahead
uint8 EVENT_GATE_PASSED=1     # node arrival — advance the route
uint8 EVENT_GOAL_REACHED=2    # green cone confirmed — stop

uint8 event
float32 distance              # meters to gate midpoint (for IN_RANGE)
