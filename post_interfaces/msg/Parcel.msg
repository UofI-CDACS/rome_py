# post_interfaces/msg/Parcel.msg

string owner_id            # Creator/owner of the parcel
string parcel_id           # Unique ID for this parcel

string prev_location       # Previous Station to hold this parcel.
string next_location       # Targeted Station to hold this parcel.
string instruction_set  # Which instruction set to use

uint64 timestamp_sent
uint64 timestamp_recieved

post_interfaces/KeyValue[] data  # Flexible metadata for state, ttl, error info, etc.
