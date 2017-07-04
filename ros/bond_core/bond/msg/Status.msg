Header header
string id  # ID of the bond
string instance_id  # Unique ID for an individual in a bond
bool active

# Including the timeouts for the bond makes it easier to debug mis-matches
# between the two sides.
float32 heartbeat_timeout
float32 heartbeat_period