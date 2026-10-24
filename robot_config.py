"""Hardware geometry and known start geometry; no route or turn schedule."""
NAMES = ('R1_Green', 'R2_Blue', 'R3_Orange', 'R4_Purple')
OFFSETS = ((0.40, 0.40), (0.40, -0.40), (-0.40, 0.40), (-0.40, -0.40))
COLORS = ((0.10, 0.72, 0.39), (0.15, 0.53, 0.95), (1.0, 0.48, 0.12), (0.70, 0.35, 0.95))
WHEEL_RADIUS = 0.045
TRACK = 0.24
RADIO_DELAY = 0.08
RADIO_TIMEOUT = 0.30
PACKET_LOSS = 0.08

# The purple follower bridges the two triangles shown in the reference.
PEER_NEIGHBOURS = {1: (3,), 2: (3,), 3: (1, 2)}
